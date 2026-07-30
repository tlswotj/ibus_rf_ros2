# ibus_rf_ros2

FlySky `FS-i6X` 송신기와 `FS-A8S` 수신기의 `i-BUS` 출력을 리눅스 시리얼로 받아 ROS 2 토픽으로 변환하는 패키지다.  
이 저장소의 ROS 2 패키지 이름은 `rf_joy`이고, 현재 패키지 디렉터리는 `src/ibus_rf_ros2`에 있다.

이 패키지는 두 단계로 동작한다.

1. `rf_publisher_node`
   `i-BUS` 시리얼 데이터를 읽어 `/rf` (`std_msgs/msg/UInt16MultiArray`)로 발행한다.
2. `rf_to_joy_node`
   `/rf`를 받아 `sensor_msgs/msg/Joy` 형식의 `/joy`로 변환한다.

기본 테스트 환경은 `Jetson Orin NX` + `FS-i6X` + `FS-A8S` + `UART(/dev/ttyTHS1)` 조합이다.

## 1. 사용 장비와 통신 개요

현재 사용 장비는 아래와 같다.

- 송신기: `FlySky FS-i6X`
- 수신기: `FlySky FS-A8S`
- 수신 방식: `AFHDS 2A`
- 수신기 출력: `PPM / i-BUS / S.BUS`

`FS-A8S`는 소형 수신기이며 공식 스펙상 `4.0V ~ 8.4V` 전원 입력과 `PPM / i-BUS / S.BUS` 데이터 출력을 지원한다.  
공식 매뉴얼 요약 기준으로 `i-BUS`는 최대 18채널, `S.BUS`는 최대 16채널 출력이 가능하며, `RSSI`가 `CH14` 값으로 전달될 수 있다.  
이 패키지는 현재 `i-BUS` 프레임에서 **14개 채널**을 읽어 `/rf`로 발행한다.

`i-BUS` 채널 값은 하위 `12bit`에 담긴다.
송신기를 18채널 모드로 두면 `CH15 ~ CH18` 값이 뒤쪽 채널의 상위 니블에 실려 오기 때문에,
이 패키지는 채널 값을 `0x0FFF`로 마스킹한 뒤 발행한다.

통신 흐름은 아래와 같다.

```text
FS-i6X (송신기)
  -> 2.4GHz RF
FS-A8S (수신기)
  -> i-BUS signal line
리눅스 UART RX 또는 USB-TTL 어댑터 RX
  -> rf_publisher_node
  -> /rf
  -> rf_to_joy_node
  -> /joy
```

## 2. 하드웨어 연결 방법

이 패키지는 `FS-A8S`의 `i-BUS/S.BUS` 출력 핀을 리눅스 UART 수신선에 연결하는 구성을 전제로 한다.

필수 연결은 아래 3가지다.

1. 수신기 전원 공급
2. 수신기와 컴퓨터/SBC/USB-TTL 간 공통 `GND` 연결
3. 수신기의 `i-BUS/S.BUS` 신호선을 리눅스 측 `RX`에 연결

배선 개념은 아래와 같다.

```text
FS-A8S VCC          ->  적절한 전원
FS-A8S GND          ->  UART/USB-TTL GND
FS-A8S i-BUS/S.BUS  ->  UART/USB-TTL RX
```

주의할 점:

- 이 패키지는 `TX`로 데이터를 보내지 않고, 수신기의 `i-BUS`를 읽기만 한다.
- 일반 PC의 레거시 `RS-232` 포트가 아니라 `TTL UART` 또는 `USB-TTL` 어댑터를 기준으로 연결해야 한다.
- 수신기 전원은 별도로 안정적으로 공급되어야 한다.
- `GND`를 공통으로 연결하지 않으면 시리얼 수신이 불안정하거나 전혀 동작하지 않을 수 있다.
- `FS-A8S`는 `i-BUS`와 `S.BUS`를 모두 지원하므로, 실제 출력 모드가 `i-BUS`인지 확인해야 한다.

## 3. FS-i6X / FS-A8S 초기 설정

### 3.1 바인딩

일반적인 바인딩 순서는 아래와 같다.

1. `FS-i6X` 전원을 끈다.
2. `FS-A8S`를 바인드 모드로 넣는다.
   보통 수신기의 `BIND` 버튼을 누른 상태로 전원을 넣으면 LED가 빠르게 점멸한다.
3. 송신기에서 `BIND KEY`를 누른 상태로 전원을 켠다.
4. 바인딩이 성공하면 수신기 LED 상태가 바뀌고, 송신기에 바인딩 완료 표시가 나타난다.
5. 송신기와 수신기를 껐다가 다시 켠다.
6. 정상 연결되면 수신기 LED가 고정되고 채널 입력이 들어온다.

정확한 LED 패턴이나 메뉴 표기는 송신기/수신기 펌웨어 버전에 따라 조금 다를 수 있으므로, 하단의 참고 문서도 함께 보는 것을 권장한다.

### 3.2 수신기 출력 모드 확인

`FS-A8S`는 `PPM / i-BUS / S.BUS`를 지원한다.  
이 패키지는 반드시 `i-BUS` 모드여야 한다.

수신기에서 `S.BUS` 모드로 출력 중이면 `/rf`가 정상적으로 나오지 않는다.  
출력 모드 전환은 `FS-A8S` 매뉴얼 또는 제품 문서에 따라 수행한다.

### 3.3 송신기 채널 권장 설정

현재 기본 설정은 아래 용도를 기준으로 작성되어 있다.

- `CH1` ~ `CH4`: `Joy axes`
- `CH5`: 킬 스위치
- `CH6`: Joy 발행 모드/버튼 입력

즉 기본 YAML에서는 다음처럼 동작한다.

- `CH5 > 1600` 이면 킬 스위치 해제 상태로 간주
- `CH5 <= 1600` 이면 `/joy`를 0으로 채워 발행
- `CH6 < 1600` 일 때만 `/joy`를 발행
- `CH6` 값으로 버튼 2개를 threshold 기반으로 생성

### 3.4 Failsafe 설정 권장

실기 테스트 전에는 송신기 failsafe를 반드시 설정하는 것을 권장한다.  
특히 킬 스위치 채널(`CH5`)은 신호 손실 시 안전한 방향으로 떨어지도록 설정하는 편이 좋다.

## 4. 설치 방법

아래 설명은 Ubuntu 22.04 + ROS 2 Humble 기준이다.

### 4.1 필수 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  libserial-dev
```

이미 ROS 2 Humble이 설치되어 있다면 `libserial-dev`와 `python3-colcon-common-extensions`만 추가로 설치하면 된다.

### 4.2 워크스페이스 배치

이 저장소는 `colcon` 워크스페이스 안에서 아래와 같은 형태를 가정한다.

```text
<workspace>/
  src/
    ibus_rf_ros2/
      CMakeLists.txt
      package.xml
      src/
      launch/
      config/
```

주의:

- 패키지 디렉터리 이름은 `ibus_rf_ros2`
- ROS 2 패키지 이름은 `rf_joy`

따라서 빌드할 때는 디렉터리 이름이 아니라 패키지 이름 `rf_joy`를 사용한다.

### 4.3 빌드

워크스페이스 루트에서 아래를 실행한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rf_joy
```

빌드 후 환경을 적용한다.

```bash
source install/setup.bash
```

lint 검사는 아래로 실행한다.

```bash
colcon test --packages-select rf_joy
colcon test-result --verbose
```

### 4.4 시리얼 권한

USB-TTL 어댑터가 `/dev/ttyUSB0` 등으로 잡혔는데 권한 에러가 나면 아래를 확인한다.

```bash
ls -l /dev/ttyUSB0
groups
```

필요하면 사용자에게 `dialout` 그룹을 부여한다.

```bash
sudo usermod -a -G dialout $USER
```

그 뒤 로그아웃/로그인 또는 재부팅 후 다시 확인한다.

## 5. 설정 파일

설정 파일은 `config/` 아래 두 개다.

- `config/rf_publisher.yaml`
- `config/rf_to_joy.yaml`

### 5.1 `rf_publisher.yaml`

파일: [`config/rf_publisher.yaml`](./config/rf_publisher.yaml)

```yaml
rf_publisher_node:
  ros__parameters:
    serial_port: "/dev/ttyTHS1"
    baud_rate: 115200
    read_timeout_ms: 50
    publish_latest_only: true
    max_publish_rate_hz: 30.0
    signal_timeout_ms: 500
    reconnect_interval_ms: 1000
    qos:
      reliability: "reliable"
      depth: 10
```

파라미터 설명:

- `serial_port`
  사용할 리눅스 시리얼 디바이스
  예: `/dev/ttyTHS1`, `/dev/ttyUSB0`, `/dev/ttyACM0`
- `baud_rate`
  `i-BUS` 수신 baud rate
  기본값은 `115200`
- `read_timeout_ms`
  데이터가 올 때까지 `poll()`로 대기하는 최대 시간(ms)
  수신이 없을 때 읽기 스레드가 커널에서 잠겨 있는 단위이고, 노드 종료 응답 시간의 상한도 된다
- `publish_latest_only`
  `true`면 한 번에 읽어들인 데이터에서 가장 최신 프레임만 발행한다 (제어 입력에는 이 값을 권장)
  `false`면 읽어들인 유효 프레임을 모두 발행한다
- `max_publish_rate_hz`
  `/rf` 발행 상한(Hz)
  포트는 항상 `i-BUS` 속도(약 `140Hz`)로 다 비우고, 이 값은 "가장 최신 프레임을 얼마나 자주 DDS로 넘길지"만 제한한다
  따라서 낮춰도 지연이 누적되지 않는다
  `0.0`이면 제한 없이 프레임마다 발행한다
  실측 발행률은 설정값보다 `5 ~ 10%` 낮게 나온다 (간격 기준이 마지막 발행 시각이라 상한을 절대 넘지 않는다)
- `signal_timeout_ms`
  이 시간 동안 유효한 `i-BUS` 프레임이 없으면 경고 로그를 남긴다
- `reconnect_interval_ms`
  포트를 열지 못했거나 읽는 중 끊어졌을 때 재시도 간격(ms)
- `qos.reliability` / `qos.depth`
  `/rf` 토픽의 QoS
  `rf_to_joy_node`의 `qos.rf_subscription`과 **반드시 같아야** 한다

> `publish_rate_hz`는 `max_publish_rate_hz`로 대체됐다.
> 이전 구현은 타이머 한 tick마다 프레임을 딱 1개만 읽었는데, `i-BUS` 프레임은 약 `7ms`마다 오기 때문에
> 남는 프레임이 커널 버퍼에 계속 쌓이다가 버퍼가 포화되면 **약 1초 지연이 고정으로 남았다**.
> 지금은 읽기와 발행이 분리되어 있어서, 포트는 항상 다 비우면서 발행률만 따로 정할 수 있다.

### 5.2 읽기 방식과 CPU 사용량

`i-BUS` 프레임 속도를 그대로 발행하면 `/rf`, `/joy` 양쪽의 `publish` / `take` / 콜백 비용이 약 5배가 된다.
그래서 이 패키지는 **읽기는 최대 속도로, 발행은 제한**하는 구조를 쓴다.

pty에 `140Hz`로 합성 `i-BUS` 프레임을 흘려 실제 `libserial 1.0.0`으로 측정한 결과다.
(x86 컨테이너 기준이므로 절대값이 아니라 비율만 참고할 것)

| 방식 | 프레임 파싱 | 유실 | `/rf` 발행 | `read()` syscall / 프레임 |
|---|---|---|---|---|
| 이전 (30Hz 타이머, 1바이트씩) | `29.8/s` | **`451/1391`** | `29.8/s` | `32` |
| 현재 (`max_publish_rate_hz: 0.0`) | `139.1/s` | `0` | `139.1/s` | `2` |
| **현재 기본값 (`30.0`)** | `139.1/s` | `0` | `28.0/s` | `2` |

핵심은 두 가지다.

- 읽기 비용이 **줄었다**. `poll()` 1회 + `read()` 1회로 큐에 있는 걸 다 가져오므로,
  프레임당 `read()` syscall이 `32`에서 `2`로 떨어진다.
  이전 방식은 프레임 대부분을 버리면서도 syscall을 더 많이 썼다.
- 발행 부하는 기본값에서 이전과 같다. `28/s`는 이전 `29.8/s`와 사실상 동일하다.

#### `libserial`의 `Read()`를 쓰지 않는 이유

`rf_publisher_node`는 `libserial`로 포트를 열고 설정하지만, 읽기는 `GetFileDescriptor()`로 얻은
디스크립터에 직접 `poll()` + `read()`를 호출한다. 계층을 건너뛰는 것처럼 보이지만 이유가 있다.

`libserial 1.0.0`의 `Read(buffer, n, msTimeout)`은 **블로킹 포트에서 타임아웃을 지키지 못한다.**
`read()` 안에서 `n`바이트가 다 모일 때까지 커널에 잠겨 있고, 자체 타임아웃 검사는 루프를 한 바퀴
돌 때만 하기 때문에 검사에 도달하지 못한다. 그래서 `GetNumberOfBytesAvailable()`로 `n`을 정하면,
큐가 알려준 것보다 실제로 적게 넘어오는 순간 **무한정 멈춘다.**

실측하면 큐에 32바이트가 있을 때 `Read(..., 32, 50)`은 `0.02ms`에 정상 반환하지만,
`Read(..., 33, 50)`은 `50ms` 타임아웃을 무시하고 영구 블로킹된다.
이 때문에 실제 UART에서 `/rf`가 약 `10Hz`까지 떨어지는 문제가 있었다. (pty 테스트에서는
`FIONREAD`와 `read()`가 정확히 일치해서 드러나지 않았다.)

`poll()` + `read()`는 두 문제가 모두 없다.

- 수신이 없으면 `poll()`이 커널에서 대기하므로 유휴 CPU가 0이다 (`read_timeout_ms`로 상한)
- 큐에 있는 만큼만 한 번에 가져오고, 오지 않을 바이트 개수를 기다리지 않는다
- 백로그도 한 번의 `read()`로 다 비운다 (측정: 5프레임 160바이트를 `0.004ms`에 일괄 반환)

CPU 여유가 있고 지연을 더 줄이고 싶으면 `max_publish_rate_hz`를 올리거나 `0.0`으로 두면 된다.
반대로 더 줄여야 하면 값을 낮추되, `rf_to_joy_node`의 `watchdog.timeout_ms`가
`/rf` 발행 주기의 몇 배는 되도록 함께 확인한다.

시리얼 장치 이름을 찾는 데는 아래 명령이 유용하다.

```bash
ls /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

USB-TTL을 연결한 직후 어떤 장치가 생겼는지 보려면 아래를 사용한다.

```bash
dmesg | tail -n 50
```

### 5.3 `rf_to_joy.yaml`

파일: [`config/rf_to_joy.yaml`](./config/rf_to_joy.yaml)

```yaml
rf_to_joy_node:
  ros__parameters:
    kill_switch:
      channel: 4
      threshold: 1600
      active_when_above: true

    publish_gate:
      enabled: true
      channel: 5
      threshold: 1600
      active_when_above: false

    axes:
      channels: [0, 1, 2, 3]
      offsets: [1500.0, 1500.0, 1500.0, 1500.0]
      scales: [500.0, 500.0, 500.0, 500.0]
      invert: [false, false, false, false]

    buttons:
      channels: [5, 5]
      thresholds: [1300, 1600]
      active_when_above: [true, true]

    watchdog:
      enabled: true
      timeout_ms: 200
      rate_hz: 50.0
      # failsafe_axes: [0.0, 0.0, -1.0, 0.0]

    qos:
      rf_subscription:
        reliability: "reliable"
        depth: 10
      joy_publisher:
        reliability: "reliable"
        depth: 10
```

중요:

- 여기서 채널 번호는 `0-based index`다.
- 즉 `CH1 -> 0`, `CH2 -> 1`, `CH5 -> 4`, `CH6 -> 5`로 적어야 한다.

파라미터 설명:

- `kill_switch`
  이 조건이 거짓이면 `/joy`는 failsafe 값으로 채워 발행된다.
- `publish_gate`
  이 조건이 거짓이면 `/joy` 자체를 발행하지 않는다.
  단 `kill_switch`와 `watchdog`은 이 게이트와 무관하게 항상 failsafe를 발행한다.
- `axes.channels`
  배열 순서가 `Joy.axes[]` 인덱스가 된다.
- `axes.offsets`
  축 중심값
- `axes.scales`
  정규화 스케일
  일반적으로 `(raw - offset) / scale`로 계산된다.
- `axes.invert`
  축 반전 여부
- `buttons.channels`
  배열 순서가 `Joy.buttons[]` 인덱스가 된다.
- `buttons.thresholds`
  버튼 판단 임계값
- `buttons.active_when_above`
  `true`면 `value > threshold`일 때 1
  `false`면 `value < threshold`일 때 1
- `watchdog.enabled`
  `/rf` 감시 기능 사용 여부
  끄면 신호가 끊겼을 때 `/joy` 발행이 그냥 멈추므로, 다운스트림이 마지막 명령을 계속 붙들 수 있다
- `watchdog.timeout_ms`
  이 시간 동안 사용 가능한 `/rf` 메시지가 없으면 failsafe 상태로 들어간다
- `watchdog.rate_hz`
  failsafe 상태에서 `/joy`를 다시 발행하는 주기
- `watchdog.failsafe_axes`
  failsafe 상태와 킬 스위치 동작 시 내보낼 축 값
  지정하지 않으면 모든 축을 `0.0`으로 채운다
  스로틀처럼 중앙(`0.0`)이 안전값이 아닌 축이 있으면 `axes.channels`와 같은 길이로 `-1.0 ~ 1.0` 범위에서 직접 지정한다
- `qos.rf_subscription`
  `/rf` 구독 QoS
  `rf_publisher_node`의 `qos`와 **반드시 같아야** 한다. 다르면 데이터가 아예 흐르지 않는다
- `qos.joy_publisher`
  `/joy` 발행 QoS
  `teleop_twist_joy` 같은 일반적인 구독자는 `reliable`을 쓰므로 기본값을 그대로 두는 것을 권장한다

`watchdog.failsafe_axes`는 빈 배열 `[]`로 적지 않는다.
ROS 2 파라미터 YAML은 빈 시퀀스의 타입을 추론할 수 없어서, 필요 없으면 항목 자체를 주석 처리해 두면 된다.

### 5.4 안전 동작 정리

`/joy` 출력은 아래 우선순위로 결정된다.

1. `/rf`가 `watchdog.timeout_ms` 이상 없으면 → failsafe 발행 (게이트 무시)
2. 킬 스위치 조건이 거짓이면 → failsafe 발행 (게이트 무시)
3. `publish_gate` 조건이 거짓이면 → 발행하지 않음
4. 그 외 → 정상 매핑 발행

`rf_publisher_node`도 실행 중 시리얼 장치가 사라지면 프로세스를 종료하지 않고
`reconnect_interval_ms` 간격으로 포트를 다시 열려고 시도한다.

## 6. 실행 방법

빌드 후 워크스페이스 루트에서 아래를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rf_joy rf_joy_launch.py
```

기본적으로 아래 두 노드가 함께 실행된다.

- `rf_publisher_node`
- `rf_to_joy_node`

다른 설정 파일을 쓰고 싶다면 launch argument로 넘기면 된다.

```bash
ros2 launch rf_joy rf_joy_launch.py \
  serial_config:=/absolute/path/to/rf_publisher.yaml \
  joy_config:=/absolute/path/to/rf_to_joy.yaml
```

## 7. 동작 확인 방법

### 7.1 raw RF 값 확인

```bash
ros2 topic echo /rf
```

정상이라면 채널 값이 대체로 `1000 ~ 2000` 부근에서 움직인다.

### 7.2 Joy 출력 확인

```bash
ros2 topic echo /joy
```

정상이라면:

- `axes`는 `-1.0 ~ 1.0` 범위로 변한다.
- `buttons`는 `0` 또는 `1`로 나온다.

### 7.3 주기 확인

```bash
ros2 topic hz /rf
ros2 topic hz /joy
```

## 8. 트러블슈팅

### `/rf`가 전혀 안 나오는 경우

- `serial_port`가 올바른지 확인한다.
- 수신기 출력 모드가 `i-BUS`인지 확인한다.
- `GND` 공통 연결 여부를 확인한다.
- 수신기에 전원이 제대로 공급되는지 확인한다.
- 송신기와 수신기가 바인딩되어 있는지 다시 확인한다.

### `/rf`는 나오는데 `/joy`가 안 나오는 경우

- `kill_switch` 조건이 현재 입력 상태와 맞는지 확인한다.
- `publish_gate.enabled`와 `publish_gate` threshold가 현재 스위치 방향과 맞는지 확인한다.
- 채널 번호를 `0-based`로 썼는지 확인한다.
- `qos.rf_subscription`이 `rf_publisher_node`의 `qos`와 같은지 확인한다.
  한쪽만 `best_effort`면 QoS가 호환되지 않아 데이터가 아예 흐르지 않는다.

### `/joy`가 계속 failsafe 값만 나오는 경우

`No /rf message for ... ms, publishing failsafe Joy` 경고가 보이면 watchdog이 동작 중인 것이다.

- `/rf`가 실제로 발행되고 있는지 `ros2 topic hz /rf`로 확인한다.
- `RF data size ... is smaller than required` 경고가 함께 보이면 채널 매핑이 프레임 폭을 넘긴 것이다.
  `axes.channels` / `buttons.channels` 값을 확인한다.
- `/rf` 발행 주기가 `watchdog.timeout_ms`보다 느리면 오탐이 난다.
  `max_publish_rate_hz`를 낮게 설정했다면 `timeout_ms`도 함께 늘려야 한다.

### `CH7 ~ CH14` 값이 비정상적으로 큰 경우

송신기가 18채널 `i-BUS` 모드일 때 나타난다.
현재 코드는 `0x0FFF` 마스킹을 하므로 정상 범위로 나와야 하며, 그래도 이상하면 송신기 출력 모드를 확인한다.

### `No valid IBUS frame ... for ... ms` 경고가 계속 나오는 경우

바이트는 들어오지만 유효한 프레임으로 조립되지 않는 상태다.

- `baud_rate`가 `115200`인지 확인한다.
- 수신기 출력 모드가 `S.BUS`가 아니라 `i-BUS`인지 확인한다.
- 신호선을 `RX`에 연결했는지, `GND`가 공통인지 확인한다.

### 축/버튼이 기대와 다르게 매핑되는 경우

- `axes.channels`와 `buttons.channels`를 다시 확인한다.
- `CH1`을 `1`로 적는 실수가 가장 흔하다.
- 중심값이 어긋나면 `axes.offsets`
- 반전이 필요하면 `axes.invert`
- 감도가 크거나 작으면 `axes.scales`

### 시리얼 권한 에러가 나는 경우

- `dialout` 그룹 설정을 확인한다.
- Jetson 내장 UART를 쓰는 경우 장치 이름이 `/dev/ttyTHS1` 같은지 다시 확인한다.
- USB-TTL을 쓰는 경우 연결할 때마다 `/dev/ttyUSB0`, `/dev/ttyUSB1`처럼 달라질 수 있다.

## 9. 참고 자료

아래 자료를 참고해 FS-i6X / FS-A8S 설명을 보강했다.

- FlySky FS-i6X 제품 설명: https://www.flysky-cn.com/i6x-gaishu-1
- FlySky FS-i6X 사양: https://www.flysky-cn.com/i6x-canshu-1
- FlySky FS-A8S 사양: https://www.flysky-cn.com/a8sspecifications
- FlySky FS-A8S 매뉴얼 다운로드: https://www.flysky-cn.com/a8smanual
- FlySky failsafe 안내: https://www.flysky-cn.com/journal/2021/2/21/about-the-flysky-remote-control-failsafe-function
- FS-A8S Quick Start Manual 요약: https://www.manualslib.com/manual/2768791/Flysky-Fs-A8s.html
- FS-i6 / FS-i6X 바인딩 절차 요약: https://www.manualslib.com/manual/3611714/Flysky-Fs-I6.html

## 10. 라이선스

`Apache License 2.0`. 전문은 [`LICENSE`](./LICENSE)를 참고한다.
