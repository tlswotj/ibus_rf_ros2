# 기여 안내

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

이 저장소의 ROS 2 패키지 이름은 `rf_joy`이고, 패키지 디렉터리는 `src/ibus_rf_ros2`다.

## 변경 전 확인

빌드와 검사는 워크스페이스 루트에서 실행한다.

```bash
colcon build --packages-select rf_joy
colcon test --packages-select rf_joy
colcon test-result --verbose
```

`colcon test`는 `i-BUS` 파서 단위 테스트(`test/test_ibus_protocol.cpp`)와
`ament_lint_common` 계열 검사를 함께 돌린다. 두 가지 모두 통과한 상태로 올린다.

`i-BUS` 프레임 파싱 로직을 건드렸다면 하드웨어 없이 검증할 수 있으므로
`test/test_ibus_protocol.cpp`에 해당하는 케이스를 같이 추가한다.

## 코드 스타일

- `ament_uncrustify` / `ament_cpplint` 기준을 따른다. 서식 문제는 아래로 확인한다.

  ```bash
  ament_uncrustify src include test
  ament_cpplint --root include src include test
  ```

- 새 소스 파일에는 기존 파일과 같은 `Apache-2.0` 라이선스 헤더를 넣는다.
- 주석은 "무엇을 하는지"보다 "왜 그렇게 했는지"를 적는다.
  이 저장소의 주석 상당수가 실제로 겪은 실패(지연 누적, 발행률 붕괴 등)의 기록이다.

## 라이선스

기여한 내용은 `Apache License 2.0`으로 배포된다.
