# RAAICON 2026 (Paper 456) 발표 영상 — 디지털 트윈 구동 녹화 계획

목표: 온라인 발표 영상(≤10분) 안에 들어갈 **라이브 트윈 실동작 클립 2~3분** 확보.
슬라이드 6(Method 2 — live bidirectional twin)과 9(Results — driving the real robot
from the twin)에 삽입. 마감: 영상 + 슬라이드 PDF를 드라이브 폴더에 **9/11까지**.

## 0. 녹화 도구 (이 PC: Ubuntu GNOME, X11, 2560×1080 단일 모니터)

| 방법 | 설치 | 비고 |
|---|---|---|
| **GNOME 내장 스크린캐스트** `Ctrl+Alt+Shift+R` | 없음 | `~/Videos/Screencast*.webm`, 30 fps, 마이크 없음. 즉시 가능 |
| `simplescreenrecorder` (권장) | `sudo apt install -y ffmpeg simplescreenrecorder` | mp4/h264, 영역 선택, 마이크·시스템소리 |
| ffmpeg 직접 | 위와 동일 | `ffmpeg -f x11grab -framerate 30 -video_size 2560x1080 -i :1 -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p twin.mp4` |

설치는 sudo가 필요하므로 프롬프트에 `! sudo apt install -y ffmpeg simplescreenrecorder` 입력.
현장(실로봇) 샷은 **휴대폰 가로 1080p**로 찍고, 시작할 때 화면 앞에서 손뼉 한 번(싱크용).

## 1. 세션 준비 (로봇 있을 때)

1. PC 와이파이: 로봇 망(**AMMR20** 192.168.10.x 또는 ammr20_test 192.168.31.x) — 둘 다 지원되게
   `~/cyclonedds_isaac.xml`(인터페이스 이름 wlp6s0, 피어 2개)과 `scripts/ammr_net_check.sh`(IP 자동탐지) 수정됨.
2. 로봇에서: `ros2 launch digital_twin_bridge digital_twin.launch.py` + **Nav2/AMCL 기동 + 초기포즈** (AMCL 갱신 없으면 pose 동결).
3. 이 PC: `scripts/ammr_net_check.sh` → PASS(30 Hz) 확인.
4. 트윈 실행 (GUI):
   `scripts/run_isaac_ammr.sh twin --map-offset 0.4201 2.3043 55.11`
   → 책상/벽 기준으로 정합 눈확인. 어긋나면 로봇을 motor_035 자리에 세우고
   `scripts/run_isaac_ammr.sh twin --anchor 0.7365 0.7815 0.809` (반대 방향이면 yaw 3.951) → 출력된 offset 재사용.
5. Isaac 뷰포트 정리: 창 최대화, 시맨틱 오버레이 보이게, 통계(FPS) HUD는 한 컷만.

## 2. 샷 리스트 (총 ~2.5분 클립, 편집 전 원본은 넉넉히)

| # | 컷 | 소스 | 길이 | 대응 슬라이드 |
|---|---|---|---|---|
| 1 | 트윈 씬 플라이오버: 실내 전체 → 객체 오버레이(라벨 색) → 벽 충돌박스 토글 | 화면 | 20 s | 4·5 |
| 2 | 실제 테스트룸 와이드샷, 로봇이 앵커 자리(motor_035) | 폰 | 10 s | 6 |
| 3 | 트윈 기동 → 첫 pose 수신 → 로봇 모델이 현재 위치로 스냅 | 화면 | 15 s | 6 |
| 4 | 수동 주행(텔레옵) → 트윈 실시간 추종. 폰+화면 **동시** 녹화 → 좌우 분할 | 폰+화면 | 40 s | 6·9 |
| 5 | Isaac에서 빨간 콘 드래그 → 실로봇 Nav2 주행 → 도착. 폰은 로봇 따라감 | 폰+화면 | 40 s | 9 |
| 6 | 도착 위치 실측(줄자/레이저) vs 트윈 좌표 — 표 III의 6.4 cm 근거 | 폰 | 15 s | 8·9 |
| 7 | 뷰포트 FPS HUD 클로즈업 (104 FPS) | 화면 | 5 s | 7 |

주의: 컷 4·5는 로봇을 **0.5 m 이상 크게** 움직여야 트윈 갱신이 눈에 보임. 콘은 자식 프림 아무 데나 잡아도 됨.

## 3. 편집 (ffmpeg, 설치 후)

```bash
# 폰(real.mp4) + 화면(twin.mp4) 좌우 분할, 높이 1080 맞춤, 손뼉 시각으로 -ss 오프셋 정렬
ffmpeg -ss 00:00:02.3 -i real.mp4 -ss 00:00:05.1 -i twin.mp4 -filter_complex \
 "[0:v]scale=-2:1080[a];[1:v]scale=-2:1080[b];[a][b]hstack" -c:v libx264 -crf 20 -t 40 cut4_split.mp4
```
최종 발표 영상 = 슬라이드 화면녹화(OBS/simplescreenrecorder, 마이크) + 위 클립 삽입 → 10분 이내 확인.

## 4. 현재 상태 (2026-09-08)

- 로봇 오프라인(핑 불가). PC는 AMMR20(192.168.10.6) 접속 중 → 로봇 켜지면 `192.168.10.66`로 잡힐 것.
- 녹화 도구 미설치 → GNOME 내장 단축키로 바로 가능, mp4 원하면 위 apt 설치.
- 트윈 소프트웨어 상태: 헤드리스 목업 스모크 테스트 결과는 런북 §6 참고.

## 5. 2026-09-08 실제 편집 (PiP, 유저 확인 "깔끔")
원본 `~/Videos/raaicon2026_raw/ammr_dt_sim.mp4`(1782×862, 81 s) + `ammr_dt_realword.mp4`(1920×1080, 60 s).
sim 26 s~ 메인, real 8 s~ 를 우하단 560×315 PiP → `~/Videos/raaicon2026_edit/ammr_dt_pip.mp4` (52 s).
```bash
ffmpeg -y -ss 26 -i ammr_dt_sim.mp4 -ss 8 -i ammr_dt_realword.mp4 -filter_complex \
 "[1:v]scale=560:315,pad=564:319:2:2:white[pip];[0:v][pip]overlay=1210:528:shortest=1[v]" \
 -map "[v]" -map 1:a -c:v libx264 -crf 18 -pix_fmt yuv420p -c:a aac -shortest ammr_dt_pip.mp4
```
