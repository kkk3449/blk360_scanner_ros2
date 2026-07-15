# AMMR 디지털 트윈 런북 (vis_n2 스캔 → Isaac Sim)

이 PC = Isaac Sim PC (Isaac Sim 5.1, `~/isaacsim`). 로봇 = **Stallentis AMMR**
(스워브 2모듈 대각배치, 차체 1244×794×720 mm, 도면
`2. Stallentis AMMR_Layout 2D Drawing_260106.pdf` + `ammr기구학` 스펙 기준).

## 산출물 (모두 `~/ammr_twin/`)

| 파일 | 내용 |
|---|---|
| `map_vis_n2_1.pgm/.yaml` | 점유격자 (654×474 @0.05 m, origin [-18.54,-16.45], e57 XY 프레임) |
| `scene_vis_n2_1.ply` | 다운샘플 전체 클라우드 (raw 프레임, floor_z=-1.003) |
| `vis_n2_world.usda` | **충돌 가능한** 벽 박스 1430개 + 바닥 (Isaac 물리용) |
| `vis_n2_tosm.usda` | TOSM 시맨틱 오버레이 — 객체 83개 세그먼트 포인트 + 환경 20만 pts + 장소 2개 (시각 전용) |
| `map_preview.png` | 점유격자 미리보기 |

시맨틱 파이프라인 산출물은 `~/Downloads/Cyclone360_data/blk360_seg/outputs/vis_n2_objects/`
(obj_*.ply, classification.csv, semanticObjects.json, relations, places, tosm_graph.json).

## 1. 파이프라인 재현 (이미 실행 완료, 2026-07-08)

```bash
cd ~/Downloads/Cyclone360_data/blk360_seg
# DBSCAN (구조면 자동 제거, eps 0.30 + giant-split): 68.5M pts -> 83 objects
.venv/bin/python scripts/extract_objects.py --input "/home/caselab/Downloads/DataSend/vis_n2 1.e57" --out outputs/vis_n2_objects
# Uni3D (industrial 30-class)
.venv/bin/python scripts/classify_objects.py --objects-dir outputs/vis_n2_objects --classes industrial
# Stage A TOSM + 관계 + 장소 + 그래프
.venv/bin/python scripts/build_semantic_objects.py --objects-dir outputs/vis_n2_objects --classification outputs/vis_n2_objects/classification.csv --map-id vis_n2
.venv/bin/python scripts/build_relations.py --input outputs/vis_n2_objects/semanticObjects.json
.venv/bin/python scripts/build_places.py --map ~/ammr_twin/map_vis_n2_1.pgm --yaml ~/ammr_twin/map_vis_n2_1.yaml --objects outputs/vis_n2_objects/semanticObjects.json --out outputs/vis_n2_objects/places.json
.venv/bin/python scripts/build_tosm_graph.py --objects outputs/vis_n2_objects/semanticObjects.json --relations outputs/vis_n2_objects/semanticObjects.relations.json --places outputs/vis_n2_objects/places.json --out outputs/vis_n2_objects/tosm_graph.json
# USD 내보내기 (floor_z=-1.003 -> --floor-offset 1.003)
.venv/bin/python scripts/build_usd.py outputs/vis_n2_objects/tosm_graph.json ~/ammr_twin/vis_n2_tosm.usda --points outputs/vis_n2_objects --environment ~/ammr_twin/scene_vis_n2_1.ply --floor-offset 1.003

cd ~/blk360_ros2_ws
# 점유격자 + 씬 PLY (범용, e57 -> pgm/yaml/ply, floor RANSAC)
/home/caselab/Downloads/Cyclone360_data/blk360_seg/.venv/bin/python scripts/build_occ_from_e57.py "/home/caselab/Downloads/DataSend/vis_n2 1.e57" --outdir ~/ammr_twin
# 충돌 월드 USD
python3 scripts/occ_to_usd_world.py ~/ammr_twin/map_vis_n2_1.pgm ~/ammr_twin/map_vis_n2_1.yaml ~/ammr_twin/vis_n2_world.usda --wall-height 2.5
```

선택: VLM Stage B(타입 검증)는 `scripts/vlm_annotate.py` — 예심 1뷰/본심 4뷰,
지금은 Uni3D 라벨 그대로 (clutter 40 / ladder 23 등, industrial 바이어스).

## 2. AMMR 모델

`src/ammr_description/urdf/ammr.urdf` — base_footprint / base_link(1244×794),
스워브 모듈 2개(각각 steer continuous z + wheel continuous y, r 0.075 / w 0.05,
RF [0.364,-0.137] / LR [-0.364,0.137]), 코너 캐스터 4개(마찰 0 구체),
base_scan(z 99.5 mm), 팔 목업(고정 시각). 최대 1.12 m/s / 조향 1.24 rad/s.

## 3. 텔레옵 테스트 (Isaac 단독) ← **지금 단계, user 검증**

```bash
# 터미널 1 — Isaac (GUI). 시스템 ROS를 source하지 않은 셸에서:
cd ~/blk360_ros2_ws && scripts/run_isaac_ammr.sh teleop
# 터미널 2 — 시스템 ROS 2 Jazzy:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- `run_isaac_ammr.sh`가 Isaac **내장** ROS2 jazzy 라이브러리를 세팅 (시스템
  /opt/ros/jazzy rclpy는 python3.12라 Isaac python3.11과 충돌 — source 금지,
  DDS 통신은 도메인만 맞으면 됨. RMW=fastdds, `ROS_DOMAIN_ID` 기본 0).
- Isaac이 구독: `/cmd_vel`(Twist), `/cmd_vel_stamped`(TwistStamped).
  발행: `/odom`, `/tf`(odom→base_footprint), `/clock`, `/joint_states`.
- 스워브 IK: 모듈별 v_i=(vx−ω·y_i, vy+ω·x_i) → 조향각=atan2, 휠속=|v|/r,
  ±90° 초과 시 반전(조향 최소화). 홀로노믹(vy) 지원 — teleop 키보드
  Shift 조합으로 스트레이프 확인 가능. 0.5 s 무신호 시 정지.
- 확인 포인트: 바퀴 회전 + 조향 모듈 회전, 전/후/횡/제자리회전, 벽 충돌.

## 4. 라이브 디지털 트윈 (실로봇 연동) — digital_twin_bridge 계약

로봇측 스펙은 `~/Downloads/DataSend/README.md` (digital_twin_bridge 패키지).
**CycloneDDS + ROS_DOMAIN_ID=56**, ammr wifi `192.168.31.56`, 이 PC `192.168.31.135`.

| 방향 | 토픽 | 타입 |
|---|---|---|
| ammr→Isaac | `/ammr/state` | nav_msgs/Odometry (map 프레임, 30 Hz) — 1순위 |
| ammr→Isaac | `/ammr/pose`, `/amcl_pose` | PoseStamped / PoseWithCov — 폴백 |
| Isaac→ammr | `/ammr/goal_pose` | PoseStamped → dt_goal_relay → Nav2 |

이 PC 세팅(구축 완료): `~/cyclonedds_isaac.xml` (README 5-2 그대로),
`run_isaac_ammr.sh`가 **twin 모드일 때 자동으로** CycloneDDS/domain56/URI 주입
(Isaac 내장 jazzy에 librmw_cyclonedds_cpp 있음 — 확인됨).
로컬 목업 테스트는 `AMMR_LOCAL_TEST=1`을 붙이면 FastDDS/domain0으로 떨어짐.

**내일(로봇 있을 때) 순서:**
1. (1회) `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`  ← 시스템 CLI 검증용
2. ammr에서: `ros2 launch digital_twin_bridge digital_twin.launch.py`
3. 이 PC에서 핸드셰이크 게이트: `scripts/ammr_net_check.sh`
   (ping → /ammr 토픽 discovery → /ammr/state 30 Hz 확인. 여기 통과 못 하면
   트윈 띄워도 소용없음 — 스크립트가 트러블슈팅 체크리스트 출력)
4. **평상시 (권장):** 최근 캘리브레이션 값으로 바로 실행 — 로봇 위치 무관:
   `scripts/run_isaac_ammr.sh twin --map-offset 0.4201 2.3043 55.11`
   (2026-07-15 재앵커 값. geumjeong_260316 맵 그대로일 때만 유효)
   **주의:** AMCL 초기포즈가 세션마다 다르게 수렴하면 저장값이 어긋날 수
   있음 (7/14→7/15에 1 m/9° 차이 발생). 띄운 뒤 책상 등 기준으로 정합을
   눈으로 확인하고, 어긋나면 재앵커.
   **맵을 새로 만들었거나 정합이 틀어졌을 때만:** 로봇을 스캔 당시
   자리(motor_035)에 같은 방향으로 세우고
   `scripts/run_isaac_ammr.sh twin --anchor 0.7365 0.7815 0.809`
   → 첫 pose 수신 시 map-offset 자동 계산·출력 (반대면 yaw `3.951`).
   → 출력된 새 값을 위처럼 `--map-offset`으로 고정 재사용.
5. 로봇 수동 주행 → 트윈 추종 확인 → 빨간 콘 드래그(콘 아무 데나 잡아도 됨)
   → 실로봇 Nav2 주행. **주의:** 로봇 pose는 AMCL 갱신 시에만 변함 — 정지
   중엔 동결이 정상, 트윈 확인은 0.5 m 이상 크게 움직여서.

**실로봇 DDS 핸드셰이크 통과 (2026-07-09).** 증상: ping OK인데 `/ammr` 토픽이
안 보임 (SPDP는 오가는데 SEDP가 완성 안 됨). 원인: ammr이 locator를 2개
광고함 — 내부망 `192.168.10.66` + wifi `192.168.31.56` — 우리 쪽 CycloneDDS가
유니캐스트(ACKNACK 등)를 도달 불가능한 내부망 주소로 보내서 토픽 정보 교환이
영원히 안 끝남. **해결: `~/cyclonedds_isaac.xml`에 `<DontRoute>true</DontRoute>`**
(같은 서브넷 주소만 사용). 이후 `/ammr/state` 30 Hz 수신 + net_check PASS.
참고: `ros2 topic list`가 계속 이상하면 `ros2 daemon stop`으로 데몬 캐시 리셋.

**검증 완료 (2026-07-08, 목업):** 앵커 캘리브레이션(오프셋 1.7685 2.0083
46.352° 자동 산출, 트윈이 앵커에 정확히 스냅) + `/ammr/state` 미러링 +
`/ammr/goal_pose` 왕복 루프 전부 통과. 실로봇 없이 재검증:
`python3 scripts/ammr_pose_mock.py` (시스템 ROS2, AMMR_LOCAL_TEST=1 트윈과 페어).

## 5. 정량 평가 (논문용): 실측 거리 vs 트윈 거리 오차

도구: `scripts/twin_eval_logger.sh` (기록) + `scripts/twin_eval_report.py`
(표/그림). 목업 검증 완료 (2026-07-13).

절차 (트윈 세션이 돌아가는 동안, 별도 터미널):
1. 바닥에 기준점 5~10개 표시, 레이저 거리계/줄자로 점 사이 거리 실측.
2. `scripts/twin_eval_logger.sh` 실행 → 로봇을 기준점에 세울 때마다
   라벨 + Enter (예: `A`) — 2초 평균 pose가 checkpoints.csv에 기록됨.
3. Isaac에서 goal을 찍으면 goals.csv에 자동 기록 + 도착 자동 감지
   (0.3 m 이상 이동 후 3초 정지). 실측 정지-위치 오차는 현장에서 따로 측정.
4. 종료(Ctrl-C) 후 `python3 scripts/twin_eval_report.py` → 최초 실행 시
   세션 폴더에 `ground_truth.yaml` 템플릿 생성 → 실측값 기입 → 재실행.
5. 산출: `eval_report.csv` (mean|e|/RMSE/max/bias) + `eval_fig.png`
   (실측-트윈 산점도 + 쌍별 오차 막대). 세션은 `~/ammr_twin/eval/session_*`.

지표 3종: ①체크포인트 쌍 거리 오차(트윈 좌표 vs 실측 — 로컬라이제이션+
캘리브레이션+맵정합 합산), ②goal 내부 오차(자동), ③goal 실측 오차(수동 기입).

## 알려진 한계 / 다음 단계

- Uni3D 라벨은 industrial 후보군 바이어스 → 필요시 Stage B VLM으로 교정.
- 트윈 모드에서 바퀴/조향은 애니메이션 안 함 (베이스 포즈만 미러링).
- 점유격자에 창문 밖 잡점 일부 포함 (벽 밖이라 주행엔 무영향).
- 실로봇 pose 토픽 이름/프레임은 AMMR 소프트웨어 확정 후 `--pose-topic`으로 조정.
