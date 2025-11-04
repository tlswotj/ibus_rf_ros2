#include <vector>
#include <deque>
#include <thread>
#include <chrono>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

using namespace LibSerial;
using namespace std::chrono_literals;

namespace {
constexpr size_t IBUS_PACKET_SIZE = 32;
constexpr uint8_t IBUS_LEN = 0x20;   // [0] = length(32)
constexpr uint8_t IBUS_CMD = 0x40;   // [1] = command
const std::string UART_PORT = "/dev/ttyTHS1";
}

// ===== checksum for a 32-byte IBUS frame =====
static inline bool verifyChecksum(const uint8_t* data, size_t sz) {
    if (sz != IBUS_PACKET_SIZE) return false;
    uint16_t checksum = 0xFFFF;
    for (size_t i = 0; i < sz - 2; ++i) checksum -= data[i];
    const uint16_t recv = static_cast<uint16_t>(data[sz - 2])
                        | static_cast<uint16_t>(data[sz - 1] << 8);
    return checksum == recv;
}

// ===== Serial open =====
static void portOpener(SerialPort &serial_port, const rclcpp::Logger& log) {
    try {
        serial_port.Open(UART_PORT);
        serial_port.SetBaudRate(BaudRate::BAUD_115200);
        serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial_port.SetParity(Parity::PARITY_NONE);
        serial_port.SetStopBits(StopBits::STOP_BITS_1);
        serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

        if (!serial_port.IsOpen()) {
            RCLCPP_ERROR(log, "Cannot open port: %s", UART_PORT.c_str());
            throw LibSerial::OpenFailed("Port open failed after configuration.");
        }
        RCLCPP_INFO(log, "IBUS receiver on %s (115200-8N1).", UART_PORT.c_str());
    }
    catch (const OpenFailed& e) {
        RCLCPP_ERROR(log, "Failed to open %s: %s", UART_PORT.c_str(), e.what());
        throw;
    }
}

// ===== Parse the last complete & valid frame from rx buffer =====
// Returns true if a frame was parsed into 'out' and erases bytes up to end of that frame.
static bool parseLatestFrame(std::vector<uint8_t>& rxbuf, std::vector<uint16_t>& out) {
    if (rxbuf.size() < IBUS_PACKET_SIZE) return false;

    // 뒤에서 앞으로 스캔: 가장 마지막 완전 프레임을 찾는다.
    // i는 프레임의 시작 인덱스
    for (int i = static_cast<int>(rxbuf.size()) - static_cast<int>(IBUS_PACKET_SIZE); i >= 0; --i) {
        const uint8_t* frame = rxbuf.data() + i;
        // 길이/커맨드 대략 체크
        if (frame[0] != IBUS_LEN || frame[1] != IBUS_CMD) continue;
        if (!verifyChecksum(frame, IBUS_PACKET_SIZE)) continue;

        // 채널 파싱: data[2..29] 14ch x uint16 (LE)
        out.clear();
        out.reserve(14);
        for (size_t k = 2; k < 30; k += 2) {
            uint16_t v = static_cast<uint16_t>(frame[k]) |
                         static_cast<uint16_t>(frame[k + 1] << 8);
            out.push_back(v);
        }
        // 이 프레임 이후의 찌꺼기만 남기거나, 재동기화를 위해 모두 지운다.
        // 여기서는 “재처리 방지”를 위해 프레임 끝까지 잘라낸다.
        rxbuf.erase(rxbuf.begin(), rxbuf.begin() + i + IBUS_PACKET_SIZE);
        return true;
    }

    // 유효 프레임 없음: 버퍼가 너무 길면 앞부분을 컷 (동기화 가속)
    constexpr size_t MAX_KEEP = IBUS_PACKET_SIZE * 6; // 최대 6프레임 분량만 유지
    if (rxbuf.size() > MAX_KEEP) {
        rxbuf.erase(rxbuf.begin(), rxbuf.end() - MAX_KEEP);
    }
    return false;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rf_publisher");
    auto log  = node->get_logger();
    auto pub  = node->create_publisher<std_msgs::msg::UInt16MultiArray>("/rf", 10);

    SerialPort serial_port;
    rclcpp::Rate rate(30.0); // 메인 루프 30Hz

    // 최신값 유지 (신호가 끊겨도 30Hz로 발행 지속)
    std::vector<uint16_t> last_channels(14, 1500);

    // 수신 바이트 축적 버퍼 (현재 주기에서 시리얼 버퍼를 비워 이쪽으로 모음)
    std::vector<uint8_t> rxbuf;
    rxbuf.reserve(IBUS_PACKET_SIZE * 8);

    auto last_rx_time = std::chrono::steady_clock::now();
    const auto idle_reset = 500ms;   // 이 시간 이상 무신호면 재동기화를 위해 입력 버퍼 정리

    try {
        portOpener(serial_port, log);

        while (rclcpp::ok()) {
            // 1) 가능한 모든 바이트를 한 번에 긁어서 rxbuf로 옮김 => LibSerial 내부버퍼 드레인
            bool any_read = false;
            while (serial_port.IsDataAvailable()) {
                uint8_t b = 0;
                serial_port.ReadByte(b);
                rxbuf.push_back(b);
                any_read = true;
            }
            if (any_read) last_rx_time = std::chrono::steady_clock::now();

            // 2) rxbuf에서 "가장 마지막 유효 프레임"만 파싱
            std::vector<uint16_t> parsed;
            if (parseLatestFrame(rxbuf, parsed) && parsed.size() == 14) {
                last_channels = parsed; // 최신 프레임 갱신
            }

            // 3) 무신호가 오래 지속되면(버퍼에 엉킨 잔여가 있을 수 있음) 강제로 입력버퍼/로컬버퍼 정리
            const auto now = std::chrono::steady_clock::now();
            if (now - last_rx_time > idle_reset) {
                rxbuf.clear();
                // 라이브러리가 지원한다면 입력 버퍼도 플러시
                try {
                    serial_port.FlushInputBuffer(); // 일부 LibSerial 구현에서 제공
                } catch (...) {
                    // 미지원일 수 있으니 조용히 무시
                }
                last_rx_time = now; // 중복 초기화 방지
            }

            // 4) 발행 (이번 주기에 새 프레임이 없어도 직전 최신값 유지)
            std_msgs::msg::UInt16MultiArray msg;
            msg.data = last_channels;
            pub->publish(msg);

            rclcpp::spin_some(node);
            rate.sleep(); // 30Hz 루프 → CPU 안정
        }

        if (serial_port.IsOpen()) serial_port.Close();
        RCLCPP_INFO(log, "%s closed.", UART_PORT.c_str());
    }
    catch (const OpenFailed& e) {
        if (serial_port.IsOpen()) serial_port.Close();
        RCLCPP_ERROR(log, "Cannot open %s: %s", UART_PORT.c_str(), e.what());
        rclcpp::shutdown();
        return 1;
    }
    catch (const std::exception& e) {
        if (serial_port.IsOpen()) serial_port.Close();
        RCLCPP_ERROR(log, "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

