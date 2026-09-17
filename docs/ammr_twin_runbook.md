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

## 6. 2026-09-08 재가동 준비 (RAAICON 발표 영상 녹화용)

- **망 변경 대응:** PC가 AMMR20(192.168.10.x)에 붙어 있어 옛 설정(31.135 고정)은 바인딩 실패했을 것.
  `~/cyclonedds_isaac.xml`을 인터페이스 **이름(wlp6s0)** 바인딩 + 피어 2개(31.56, 10.66)로 변경(백업 `.bak_20260908`),
  `ammr_net_check.sh`는 두 IP를 자동 탐지. 로봇이 AMMR20에 있으면 eno1 주소 192.168.10.66으로 잡힘.
- **헤드리스 목업 스모크 PASS** (`~/ammr_twin/smoke_20260908/`): 트윈 기동 → /ammr/twin_pose 발행 →
  goal(1.0,1.5) 발행 → 목업 수신·주행 → 트윈 추종((-0.41,0.93)→(-0.24,3.91)). 기동 ~15 s(헤드리스).
  `PhysicsUSD: CreateJoint - cannot create a joint between static bodies` 에러 12줄은 트윈 모드의 물리 스트립
  부작용으로 7월 스모크 로그에도 동일하게 있음 — 무시.
- 녹화 계획·샷 리스트: `docs/raaicon2026/presentation/recording_plan.md`.
- **2026-09-08 재앵커 결과:** 저장 오프셋(0.4201 2.3043 55.11)으로 띄우니 트윈이 맵 상단 밖(y 7.44 > 경계 7.25).
  로봇을 motor_035 자리에 세우고 `--anchor 0.7365 0.7815 0.809` → **CALIBRATED map-offset 3.3977 2.4861 43.119**
  (7월 값과 3 m/12° 차이 = AMCL 초기포즈 차이). 이 세션 재기동 시 `--map-offset 3.3977 2.4861 43.119`.
  → 추종 방향이 반대로 확인되어 yaw 3.951로 재앵커: **CALIBRATED map-offset -1.7625 -0.7828 -136.627** (이 값이 유효, 위 43.119 값은 폐기).

## 7. 시맨틱 웹 UI ↔ Isaac 연동 (2026-09-14, `scripts/isaacsim_kg_twin.py`, 모드 `run_isaac_ammr.sh kg`)

- 스택: `scripts/bringup_all.sh`(Gazebo visn2_room → AMCL → Nav2 → mediator+BT.CPP → ui_server:8080 → kg_to_usd --watch) 를 먼저 올리고
  `DISPLAY=:1 setsid nohup scripts/run_isaac_ammr.sh kg > ~/bringup_logs/isaac_kg.log 2>&1 &` (로컬 FastDDS/도메인 0, 기동 ~80 s).
- Isaac 씬 = `t4_kg_scene.usda`(KG에서 생성, map 프레임, 바닥 z=0) 참조 + AMMR URDF 키네마틱 트윈 + 돔/태양광 + 룸 중심 오버헤드 카메라.
- 로봇 미러: `/tf` map→odom(AMCL) ∘ `/odom`(Gazebo) 합성 → 부드러운 추종; `/amcl_pose`(transient_local)는 폴백. 오프셋 0 (같은 프레임).
- KG 핫리로드: UI에서 refute/타입/JSON 편집 → KG json 저장 → watcher가 usda 재생성(~5 s) → Isaac이 mtime 감지 후 `Sdf.Layer.Reload` (검증: touch 후 로그 "KG scene reloaded").
- 빨간 콘 드래그 → `/goal_pose`(map) → Nav2가 Gazebo 로봇 주행 → UI 로그·Isaac 트윈 동시 추종. 객체 클릭 → "TOSM Object Info" 패널(customData).
- 종료: `pkill -f "isaacsim_kg[_]twin"` (대괄호로 자기-킬 회피), 스택은 `scripts/kill_all.sh` + ui_server/mediator/mission_bt_cpp/kg_to_usd 수동 종료.
- 함정: 명령줄에 스크립트 파일명 리터럴이 들어간 상태에서 `pkill -f 이름` → exit 144 자기-킬. 패턴에 `[_]` 넣기.

## 8. 실로봇 end-to-end 시맨틱 미션 (2026-09-14 준비, Electronics R2 대응)

Gazebo/Nav2 없이 **실제 AMMR**로 UI→mediator→BT→로봇 주행, Isaac 트윈 동시 추종.
- 새 노드 `scripts/real_robot_nav_bridge.py` (도메인 56/CycloneDDS): `/ammr/state`(로봇 맵) → SE(2) 오프셋 → `/amcl_pose`(KG 프레임, latched);
  `navigate_to_pose` 액션 서버 → 역오프셋 → `/ammr/goal_pose`; 도착 = 반경 0.35 m + 정지 1 s, 타임아웃 150 s, 8 s마다 goal 재전송(릴레이 edge-trigger 대비), cancel 시 현재 위치를 goal로 보내 정지.
  `--map-offset`(기본 9/8 값 -1.7625 -0.7828 -136.627) 또는 `--anchor 0.7365 0.7815 0.809|3.951`(motor_035 자리에서 첫 pose로 캘리브레이션).
- 원스톱: `scripts/bringup_real.sh [--anchor ...]` = Gazebo 스택 정리 → net_check → 브리지 → mediator(gated)+BT.CPP → kg_to_usd --watch + ui_server:8080 → Isaac `run_isaac_ammr.sh kg --pose-source amcl`(AMMR_REAL=1 → cyclone/56). `NO_ISAAC=1`로 Isaac 생략.
- **목업 검증 통과 (9/14)**: 도메인 0에서 `ammr_pose_mock.py` + 브리지 + mediator + BT + UI → UI `goto_object tv` → BT → 브리지 GOAL KG(3.09,1.91)→robot(-5.37,1.38) → 목업 주행 → ARRIVED d=0.05 m 12.7 s → BT 완료. 스크립트: scratchpad/mock_e2e.sh (heredoc을 Bash 명령줄에 넣으면 kill_all의 pkill이 자기 셸을 죽임 → 파일로 실행).
- 현장 순서: ① 로봇: digital_twin.launch.py + Nav2/AMCL 초기포즈 ② 로봇을 motor_035 자리에(반대 방향이면 yaw 3.951) ③ `scripts/bringup_real.sh --anchor 0.7365 0.7815 3.951` ④ UI에서 객체·장소 미션, Isaac에서 콘 goal, refute 클로즈드루프 ⑤ 도착 위치 줄자 실측(논문 표).
- **9/14 현장 결과**: 앵커 캘리브레이션 후 방향이 반대로 나와 브리지 오프셋을 앵커 기준 180° 회전으로 교정(`--map-offset -3.3769 -5.8583 85.831`, 유저 텔레옵으로 방향 확인). 첫 goal 1건은 도착(0.21 m/23.8 s)했으나 이후
  `goto_object tv`는 로봇이 안 움직임 → 원인 = 로봇 쪽 **Nav2 꺼짐**(`/goal_pose` 구독자 없음; 텔레옵 `teleop_twist_keyboard`→`/manual_vel` 켜져 있었음). 내일 재개 시: 로봇 Nav2/AMCL 기동 확인 → `ros2 topic info -v /goal_pose`에 bt_navigator 구독 확인 → 미션.
- 추가된 것: UI **STOP 버튼**(`{"cmd":"stop"}` → BT 미션 취소 + `/semantic_stop` → 브리지가 현재 위치를 goal로 보내 정지), Isaac 콘 → `/kg_goal_pose` → 브리지 프레임 변환(로봇 `/goal_pose`에 KG 좌표가 직접 가던 위험 제거).
- 보조 스크립트(파일로 실행해야 pkill 자기-킬 없음): `scripts/shutdown_all.sh`, `scripts/restart_bridge.sh <args>`, `scripts/restart_isaac_real.sh`, `scripts/mock_e2e.sh`.
- 오프셋은 AMCL 초기화마다 달라짐 → 매 세션 `bringup_real.sh --anchor ...` 후 텔레옵 0.5 m로 방향 확인이 표준. 반대면 `restart_bridge.sh --anchor 0.7365 0.7815 <다른 yaw>` 또는 위 180° 회전 계산.

## 9. 2026-09-15 실로봇 시맨틱 미션 결과 (Electronics R2용)
- 요약 CSV: `~/ammr_twin/eval/semantic_missions_20260915_summary.csv` (원본 `semantic_missions_20260915.csv`, 브리지 로그 `~/bringup_logs/bridge.log`).
- **성공 3건**: goto_object tv 14 s/0.34 m (M1), goto_object tv 20.8 s/0.33 m (M8, 촬영), goto_place electrical_maintenance_area(모니터 존) 22.1 s/0.22 m (M9, 촬영). 장소 내부 즉시완료 1건(M2). **팬텀 거부 2건**(keyboard → mediator REFUSED, 로봇 정지).
- **실패 패턴**: 로봇 재시작 후 첫 미션은 항상 성공, 두 번째 목표부터 로봇이 안 움직임(우리 쪽 목표 해석·전송은 정상 → 로봇 내비 스택의 연속 goal 처리 문제; 1 m 짧은 목표도 실패, AMCL 정상). 텔레옵 사용 후에도 동일. 대응 = 미션마다 로봇 재시작(초기포즈 → 테이프 자리 → `restart_bridge.sh --anchor 0 0 -2.339`).
- 확정 사실: 로봇 보고 yaw는 실제 전면과 180° 반대(브리지 `--heading-offset 180` 기본), `/ammr/state`=로봇 AMCL과 동일 프레임, 로봇 Nav2는 도메인 56에서 안 보임(dt_goal_relay가 중계), `dt_state_publisher`가 `/amcl_pose`를 구독하므로 우리 pose는 `/kg_robot_pose`로 발행.
- 시작 자리(테이프) = KG (0,0), 실제 전면 −134°(anchor yaw −2.339 rad). Isaac 콘은 첫 pose에서 로봇 위에 자동 배치.

## 10. 콘솔 지도 패널 · 초기포즈/목표 · 브리지 옵션 (2026-09-16)

- `ui_server`에 점유격자 지도 패널 추가 (`-p map_yaml:=…`, 기본 `~/ammr_twin/map_vis_n2_1.yaml` = KG 프레임). `/map.png` 정적 배경 + 캔버스 오버레이(모든 객체: 검증=색, 미검증=회색; 선택 장소 셀; 로봇 위치+헤딩). 로봇 카메라 패널은 제거.
- 버튼 **set initial pose / send goal**: 지도에서 누르고(위치) 끌어서(헤딩) 놓으면 `POST /api/initialpose` → `initialpose`(PoseWithCovarianceStamped), `POST /api/goal` → `goal_pose`(PoseStamped). 시뮬은 Nav2 토픽 그대로; 실로봇은 `bringup_real.sh`가 `goal_pose:=/kg_goal_pose`, `initialpose:=/kg_initialpose`로 리맵.
- 브리지: `/kg_initialpose`(KG 프레임) → 로봇 프레임 변환 → `/ammr/initialpose` 발행(`--init-topic`). **로봇 측에 `/ammr/initialpose → /initialpose` relay가 아직 없음** (dt_goal_relay와 같은 방식으로 추가 필요). `--stop-mode hold|none`: STOP 시 현 위치 hold goal 재전송 여부(2번째 임무 무응답 원인 분리용).
- 테스트: `ROS_DOMAIN_ID=77`에서 `ui_server -p port:=8089` + latched `/amcl_pose` 목업 → `/api/state`에 map/robot_pose, `/api/goal`→`/goal_pose`, `/api/initialpose`→`/initialpose` 확인. 브리지 `/kg_initialpose (0,0,−134°)` → `/ammr/initialpose (−1.82, 0.64, −177°)` 확인.

## 11. 로봇 AMCL 지도 업로드 → KG 프레임 자동 정합 (2026-09-17)

- 콘솔 왼쪽 "Robot map (AMCL) → KG frame" 카드에 로봇 지도 `.pgm + .yaml`을 끌어 놓거나 선택 → `POST /api/robotmap`(multipart) → `semantic_nav_bt/map_register.py`가 KG 점유격자에 정합(360° yaw × FFT 상관 → 챔퍼 점수 → ICP; 목업 검증: 오차 1 cm / 0.04°, 2 s) → `~/ammr_twin/robot_map_offset.json` 저장 + `/kg_map_offset`(latched String JSON) 발행 → 브리지가 즉시 오프셋 갱신(`--anchor` 대기 중이면 무시). 브리지는 시작 시 `--map-offset` 미지정이면 `--map-offset-file`(기본 위 json)에서 읽음.
- 정합된 로봇 지도는 지도 패널에 주황 점으로 오버레이(체크박스). 잔차 mean/inlier가 카드에 표시.
- 의미: 오프셋이 세션 상수가 되므로 테이프 앵커 없이 아무 위치에서 시작 → 콘솔 initpose 클릭 → (로봇 측 `/ammr/initialpose→/initialpose` relay 필요) → 임무.

## 12. 콘솔 원클릭 "E57 → 3D 시맨틱 모델링" (2026-09-17)

- 카드 "3D semantic modeling from E57": `.e57` 드롭/선택 → `PUT /api/e57?name=` 스트리밍 저장(`~/ammr_twin/scans/`), 또는 서버 경로 직접 입력. 이름·모드(new site / test-room epoch)·VLM 사용+예산(USD)·오버헤드 패스 선택 → **Start** → `POST /api/pipeline/start` → `blk360_seg/scripts/semantic_pipeline.py --job <objects_dir>/job.json` (SEG venv, 별도 세션). 상태는 `pipeline_status.json`을 3 s마다 폴링(단계 ✓/▶/✗, 로그 40줄, 누적 비용). stop = 프로세스 그룹 SIGTERM.
- 단계: extract_objects(--save-clean) → classify_objects(industrial) → build_semantic_objects → [new: build_occ_from_e57 → make_room_bounds | epoch: register_scan(FPFH+ICP)] → roomscope_any → render 4+8+zoom → vlm_late_fusion(--select 실내, 예산 초과/비활성 시 skip → 잠정 라벨 미검증 편입) → kg_upsert(new: `outputs/<name>_kg.json`, epoch: `testroom_epochs_kg.json` 백업 후) → place_slic_segment + place_ring_naming(`<name>_slic`) → [overhead: overhead_extract(conn 0.15) → 어휘 분류 → 렌더(-30°) → vlm(후보·높이 사전) → upsert].
- 새 헬퍼: `register_scan.py`(CLI), `roomscope_any.py`(임의 T·bounds·tol), `make_room_bounds.py`(지도 최대 자유영역 → bounds json). 기존 `register_epoch_scan.py`/`roomscope_transform.py`의 하드코딩 경로 대체.
- "load result into console": ui_server의 kg/places/naming/map 경로를 결과로 교체(지도 패널 재로드). mediator/BT는 같은 경로로 재시작 필요. `kg_to_usd.py`는 아직 경로 상수라 Isaac 반영은 수동.
- 비용 가드: 실내 객체 수 × 13 호출 × $0.0097 추정이 예산을 넘으면 Stage B를 건너뜀(일일 ₩30,000 규칙).
