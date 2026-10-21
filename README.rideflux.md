# RideFlux Humble 패치

upstream `3.1.13` (`69b50c6db906b2044502b00c1b35dc97ec8a40ce`) 기반입니다.
`republish`의 출력 플러그인을 노드 정리가 끝날 때까지 유지해 SIGINT 종료 충돌을 수정합니다.
RideFluxSW는 이 저장소의 특정 커밋을 서브모듈로 고정합니다.

`image_transport`만 빌드하며 나머지 패키지는 `COLCON_IGNORE`로 제외했습니다.
공개 API와 패키지 버전은 유지하고, 기존 Humble 이미지의 영상 플러그인을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select image_transport --allow-overriding image_transport
source install/setup.bash
ros2 pkg prefix image_transport
colcon test --packages-select image_transport --ctest-args -R republish_shutdown --output-on-failure
colcon test-result --verbose
```

출력 경로가 workspace의 `install` 아래인지 확인합니다.
`republish_shutdown`은 설치된 `compressed_image_transport`가 있을 때 CMake에서 등록됩니다.
플러그인을 나중에 설치했다면 `colcon build --cmake-force-configure`로 다시 구성해야 합니다.
이 플러그인은 `image_transport`에 의존하므로 순환 의존성을 피하기 위해 `test_depend`에 넣지 않습니다.
검사는 실제 JPEG 전달, 노드 콜백 실행 준비, SIGINT 이후 정상 종료를 확인합니다.

RideFlux x86 이미지에서 compressed와 ffmpeg 종료를 각각 8회 검사하려면:

```bash
python3 src/RideFluxSW/ros2_custom_pkgs/image_common-humble/image_transport/test/smoke_republish_shutdown.py --transport all
```

기본 실행 파일은 현재 source한 workspace에서 찾습니다. `--republish-bin`으로 비교할 원본 실행 파일을 지정할 수 있습니다.
ffmpeg 검사는 시작·종료 수명만 확인하며 영상 디코딩 검사는 compressed/JPEG로 수행합니다.

GCC의 `GNU_UNIQUE` 심볼은 공유 라이브러리를 메모리에 남겨 종료 순서 문제를 가릴 수 있습니다.
수정 전후 비교에서는 별도 workspace에서 아래 옵션으로 빌드한 뒤 위 검사를 실행합니다.
원본 3.1.13에서는 SIGINT 후 `-11`이 재현되고 수정본에서는 정상 종료해야 합니다.

```bash
colcon build --packages-select image_transport --allow-overriding image_transport \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fno-gnu-unique
```
