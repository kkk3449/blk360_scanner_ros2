# electronics-4525706 Revision 1 — Master Plan

Reviews received 2026-09-01 (R1: methods "must be improved"; R2: design+methods "must be improved"; R3: intro/design "yes", rest "can be improved").
Response letters follow Noble's *Ten Simple Rules*: overview first, every point answered, direct answer up front, quote what changed and where, self-contained responses, polite + "we agree / accept the blame" framing.

Legend: [TEXT] text-only revision · [DATA] new analysis from existing artifacts ($0) · [EXP] new computation (geometric $0 / VLM $) · [DECISION] needs user input · [LIMIT] respond as acknowledged limitation.

## Reviewer 1

| # | Comment | Action | Plan |
|---|---|---|---|
| 1.1 | RANSAC stopping, floor/wall/ceiling identification, normalization | [TEXT] | §4.1(Methods)에 수치 명세: RANSAC 반복수·인라이어 임계·정지 조건, 평면 역할 판정 규칙(법선·z-위치), 정규화(바닥 z=0 정렬, yaw 정렬). 소스 = blk360_seg backbone 코드에서 실제 값 추출 |
| 1.2 | Structure-remnant filter numeric thresholds | [TEXT] | structure_filter.py / structure_noise_filter.py의 실제 임계값 전부 표/문단화 (띠 두께, wall-hug 비율 0.25/2m, z-span 2.5m, 천장 바닥+2m, 길이>2m 예외 등) |
| 1.3 | Exact confidence-weighted fusion rule + why 0.6 | [TEXT+DATA] | 융합 규칙 수식화(§4.4) + **0.6 임계 감도 곡선**: 저장된 per-view vote/score 로그에서 threshold sweep 재채점 (VLM 재호출 없음, $0) → 그림or표. R2.2와 공유 |
| 1.4 | Ablation without Uni3D provisional label in VLM context | [EXP $] | 신규 VLM 런: showroom 50-set(또는 대표 subset), 프롬프트에서 Stage-A 라벨 제거 조건 vs 원본 비교. 예상 ~$3–4 (일일한도 내). **유저 승인 필요** |
| 1.5 | Matching conflict resolution + 0.5m/30%/6m 근거 | [TEXT+DATA] | kg.py 매칭 로직 문서화(다중 후보 시 선택 규칙 — 코드 확인 후 서술; 없으면 규칙 명시적으로 추가), 임계 선택 근거(로봇/객체 스케일) + 결정론 replay로 임계 감도 체크 ($0). R2.4와 공유 |
| 1.6 | DBSCAN sweep circularity (operating point as reference) | [DATA] | **owner GT 기반 object-level reference로 sweep 재평가** (기하 전용 재실행, $0) — recall/precision vs owner GT로 재산출, 순환성 해소. 기존 sensitivity_*.csv 재계산. R2.3과 공유 |
| 1.7 | GT anchoring bias, blinded annotation | [LIMIT+DECISION] | 이미 §5.1에 앵커링 한계 명시됨 → 강화: owner-refutation 사례(제안 라벨 뒤집기 9건+)가 anchoring 하한 증거. 블라인드 2차 주석자 = 유저 "할 사람 없어"(8/7) → 불가 확인 필요. 가능 대안: 소규모 subset을 시간차 재라벨? |
| 1.8 | Corrective steps (re-split, ensemble, owner correction) → Methods | [TEXT] | §4에 정식 절 신설(트리거 조건·파라미터 포함), §5는 결과만. R2.8과 동일 — 가장 큰 구조 수술 |

## Reviewer 2

| # | Comment | Action | Plan |
|---|---|---|---|
| 2.1 | Distinguish new algorithms vs integration; quantify over [30],[13–15] | [TEXT] | 기여 재서술: 신규(구조적 게이팅, 신뢰티어 유지보수, mint-once ID, wall-compound gate, ensemble-as-discovery) vs 재사용(DBSCAN, Uni3D, TOSM 스키마). 정량 = layered ablation 28→85%가 이미 있음 + DK-SMF 78–81% 대비표 인용 |
| 2.2 | Fusion rule 미제시, 0.6 미분석, cafe 만장일치 57.1% scene-dependent | [TEXT+DATA] | =1.3 + scene-dependence 정직 서술(tab:cafe가 이미 근거) + threshold sweep을 양 씬에서 |
| 2.3 | Table 4 백본 검증 부적절 (proxy metric, PTv3 비교불가, sweep 순환) | [TEXT+DATA] | =1.6 (owner-GT 기반 재평가) + Table 4 캡션/본문에 proxy 한계 명시, PTv3 행은 "semantic-only, not instance-comparable" 명확화 or 분리 |
| 2.4 | Matching conflict resolution 미명세 + 3-epoch matching accuracy/ID-switch/false-new/false-absent 통계 부재 | [DATA] | =1.5 + **실측 3-epoch 매칭 성적표**: rev1→2→3 diff를 owner GT 대조로 채점 (T3 통제변화 2/2 + scenario D 데이터 있음; T2→T3 전체 매칭 채점 신규 산출, $0) |
| 2.5 | zero-cost 재검증이 controlled 시나리오만; 실제 T2→T3 아님 | [TEXT] | 정직 재스코핑: md5-carry는 동일품질 재스캔 전제(이미 문서화됨) — 주장 문구를 "controlled re-scan"으로 한정 + 실제 T2→T3 재검증 비용($) 수치 제시 |
| 2.6 | Table 5 통계력 부족, robot hall 환각 40.6%, Table 7 51% precision/31% type | [TEXT+DATA] | Table 5에 CI 추가(Wilson, tab:verification 방식 재사용), 환각·정밀 수치는 이미 정직 서술 → 스코프 문장 강화 + "상류 라벨오류 반영" 논지 유지 |
| 2.7 | Sample-flow table (38/50/41/35/46 sets 연결) | [DATA] | **신규 표**: 씬×에폭별 클러스터 흐름(원시→필터→룸스코프→GT채점→KG present) 전수 맵. 메모리+json에서 재구성, $0 |
| 2.8 | Corrective modules를 Methods로; frozen pipeline 독립 평가 | [TEXT+DATA] | =1.8 + **det5 frozen-chain 재현 런**(이미 있음: fresh full chain 28→85%, $4.2/run) + cafe held-out(구조필터 동결 적용 2/28, 오탐0) = 독립성 근거로 승격 서술 |

## Reviewer 3

| # | Comment | Action | Plan |
|---|---|---|---|
| 3.1 | dataset/benchmark 주장 과함, raw scan 미공개 | [DECISION+TEXT] | 주장 완화("dataset contribution" 문구 제거/한정). raw E57 공개 여부 = **유저 결정** (1–1.3GB×3; Zenodo 가능). supplementary(GT·쿼리셋·설정)는 이미 공개 |
| 3.2 | 결정론이 OS/버전/하드웨어 교차 미증명 | [TEXT(+EXP?)] | 주장 축소: "fixed seeds+versions, single-machine determinism 검증" 한정 + pinned requirements. (교차머신 검증은 다른 PC 있으면 옵션) |
| 3.3 | 쿼리 게이트 데이터셋 소규모 | [LIMIT] | 인정 + Wilson CI + "diagnostic, not benchmark" 스코프 (2.6과 공유) |
| 3.4a | 다중 건물 테스트 필요 | [LIMIT] | 8층 카페(다른 층·다른 기능 공간)가 stress scene으로 존재 — cross-building은 한계로 인정, jungdo E57(46.2M, GT 불가) 언급 여부 검토 |
| 3.4b | 클래스 분포 + per-class 결과 | [DATA] | 씬별 클래스 분포표 + per-class(또는 클래스군) 티어/정확도 — 기존 GT json에서 산출, $0 |
| 3.4c | 형식 정의 통합(레코드·매칭·신뢰상태·upsert) | [TEXT] | §4에 통합 형식 정의(Def. 박스 or 수식 묶음): 레코드 튜플, match(), 상태기계, upsert 규칙 — 1.5/2.4와 시너지 |
| 3.4d | 임계값이 어느 데이터로 정해졌나 | [TEXT] | 명시: T1/T2(개발 씬)에서 고정 → cafe는 동결 적용 (held-out) — §5.1 Controls에 추가 |
| 3.4e | 블라인드 GT | [LIMIT] | =1.7 |
| 3.4f | registration noise·voxel resolution·color variation 통제실험 | [DATA+EXP] | voxel/노이즈 = 기하 전용 스윕($0, 백본 재실행). color/렌더 = **기존 render-source 2×2 ablation이 정확히 이것** → 재프레이밍으로 커버. 필요시 노이즈 주입 1종만 추가 |
| 3.4g | per-tier Recall/F1 부재 | [DATA] | 기존 GT로 tier별 recall/F1 산출, tab:verification 확장, $0 |
| 3.4h | task success/path length/latency/safe stopping 메트릭 | [DATA] | §6 실측(9/9 미션, 레그 거리·시간, 팬텀 3형태, 배터리 프리엠션)을 표준 메트릭 표로 재구성, $0 |
| 3.4i | 실로봇 E2E 검증 부재 | [LIMIT] | 한계 인정 + Gazebo 실증 스코프 명확화 (fig 캡션 포함, 3.7e와 연결) |
| 3.5 | manipulation 테스트 | [LIMIT] | pick은 스코프 외(BT 스텁만) — 명시 |
| 3.7a | Table 2 셀별 근거 출처 | [TEXT] | 셀에 인용/§ 참조 각주 부여 |
| 3.7b | Table 6에 질문 수·카테고리·기권 영향 | [TEXT+DATA] | 표에 n·카테고리 열 추가 + 기권 처리 규칙 명시 (48/32셋 카테고리 분해 기존 json) |
| 3.7c | Fig 17 하단 패널 error bars | [DATA] | cafe_fig.py 막대에 CI/±표시 추가 (n 작음 → Wilson or 명시적 count 라벨) |
| 3.7d | Fig 18 결정 변화 노드 하이라이트 | [TEXT+DATA] | (v28 번호 확인 필요) 해당 그림에서 framework가 결정을 바꾼 노드 시각 강조 |
| 3.7e | Fig 19 캡션: 실로봇 검증 대체 아님 명시 | [TEXT] | 캡션+본문 한 문장 |

## 진행 상태 (2026-09-01)

- [x] 코드 파라미터 전수 추출 → `code_params_extracted.md` (file:line 근거, 함정 7건 포함)
- [x] R1.1 §4.1 재작성: RANSAC 정지 4조건·5cm 임계·평면 분류 |n_z| 규칙 명시; **허위 주장 2건 수정** (z=0 정규화는 실행 안 됨 → 스테이지별 floor 추정으로; 4cm→5cm)
- [x] R1.2 remnant filter 5-테스트 수치 명세 (0.6m/1.5m/|n_z|≤0.25/3cm/±0.3m/40pts/60%/20%)
- [x] R1.3 융합 규칙 수식화 (s(ℓ)=Σc_v, share=정규화 신뢰질량) + 0.6 a-priori 근거 + status-only 게이트 명시
- [x] R1.5/R2.4 매칭: per-axis 허용오차 공식, greedy 충돌 해소 규칙, 임계값 물리적 근거, T1/T2 고정 명시
- [x] 컴파일 확인 (34pp, 에러 0; sec:corrective 전방참조는 신설 예정 절)
- [x] §4 sec:corrective 신설 (5 레이어 정의: 트리거+파라미터) + Formal summary (R3.4c: 레코드 튜플·티어 집합·매칭 함수·업서트 케스케이드) + §5 상호참조 정리 (R1.8/R2.8 구조 수술)
- [x] Threshold sweep 완료 및 §5.3 "Threshold sensitivity" 문단 반영 — 0.6 = 플래토 (0.55-0.60 동일 티어, F1 최적 밴드 0.45-0.65; outputs/threshold_sweep.{json,csv})
- [x] 3-epoch 매칭 성적표 완료 및 §5.6 "Matching scorecard" 문단 반영 — **ID 스위치 0**, strong-pair 정확도 2/4·4/7, 오류 전원 fragmentation(extent 게이트), 충돌 pass1 0회/pass2 1회, false-new/absent 2/2·3/3 (outputs/epoch_matching_scorecard.json)
- [x] R2.5 zero-cost 재검증 재스코핑 4곳 (§1/§4/§5.6/§7): 3.1%는 동일품질 통제 재스캔 한정, 실제 T2→T3는 해시 캐리 0 + 풀 비용($3.54) 명시
- [x] R3.7a Table 2 셀별 근거 (섹션 참조 + 정의적 속성 인용), R3.7b Table 6 (n·카테고리·기권 행+캡션), 벤치 통계력 스코프 문단 (R2.6/R3.3), R3.7e Fig19 캡션 실로봇 대체 아님
- [x] 블라인드 제3자 라벨링 키트 생성·전달 (T3 41/41, 768px 풀해상 재연관 렌더, PDF+안내+CSV; 채점 키 outputs/blind_kit/id_mapping.json — 유저가 라벨러에게 전달 예정)
- [진행중] R1.4 Uni3D-blind ablation VLM 런 (claude-sonnet-4-6, --no-provisional 플래그 구현: SYMBOLIC_TOOL_BLIND/_SYS_SYMBOLIC_BLIND; 1차 런은 views-dir 오류로 esc 뷰 누락 → 16객체 후 중단(~$1.3 손실), s2_split_hdviews로 재기동+base votes 재활용)
- [진행중] Docker 결정론 키트 (4070 노트북용) — 에이전트 작업 중
- [진행중] Sample-flow/클래스분포/per-tier R·F1 표 — 에이전트 작업 중
- [x] R2.1 기여 재서술 (§1 novelty-scoping 문단: 통합 vs 신규 + 정량화), R2.8 frozen-pipeline 독립성 (§5.7 "independent run on a non-independent scene" + cafe held-out)
- [x] R1.6/R2.3 owner-GT 스윕 재평가 — **중대 발견: showroom 원본 스윕이 실제 순환(자기 클러스터 조인, recall 1.0은 아티팩트)** → §5.2 정정 서술; robot hall은 비순환 box-reference에서 운영점 그리드 최대(0.952) 확인. Table 4 캡션도 proxy·PTv3 비교불가 명시
- [x] R3.4f voxel/noise 스윕 — §5.8 신설 문단: voxel은 (ε,minpts)와 결합 캘리브레이션 (0.02/0.05에서 0.36/0.33 붕괴), 노이즈 σ5mm(실제 번들오차 스케일) recall 0.48, σ20mm 0.07, 지배 기전 = RANSAC 인라이어 밴드 포화
- [x] R1.4 ablation 완료·반영 (§5.8 Provisional-label ablation: 88.9→38.9%, Stage-B=가설 검증기)
- [x] R3.7c Fig17 Wilson 에러바, R3.7d Fig18 KG-결정 노드 하이라이트 (+캡션)
- [x] 응답서 3부 초안 (revision1/response_reviewer{1,2,3}.md) — 잔여 PENDING: 블라인드 답안 분석, 노트북 체크섬, Zenodo DOI
- [x] Data Availability에 Zenodo 문구(DOI 자리) + 결정론 키트 언급; 스펠체크 클린; main 35pp/mdpi 41pp 클린 컴파일
- [참고] R1.5 "언어 다듬기"는 RAAICON 리뷰에서 잘못 이월된 항목 — Electronics R1에는 없음

## 실행 순서(안)

1. **W1 텍스트 대수술**: 1.8/2.8 교정 모듈 →Methods 이동 + 3.4c 형식 정의 통합 + 1.1/1.2/1.3/1.5 수치 명세 (원고 구조 확정이 먼저)
2. **W2 무비용 데이터 산출**: threshold sweep(1.3), owner-GT sweep 재평가(1.6/2.3), 3-epoch 매칭 성적표(2.4), sample-flow table(2.7), per-class 분포(3.4b), per-tier R/F1(3.4g), §6 메트릭 표(3.4h), voxel/noise 기하 스윕(3.4f), Table5 CI(2.6), fig 수정 3건
3. **W3 유료 실험**: Uni3D-label ablation(1.4, ~$3–4, 승인 후)
4. **W4 응답서**: reviewer별 Ten-Rules 포맷 (overview → point-by-point, 변경 위치 명시)

## 에디터 트랙 (Ms. Yan 메일 5건, 9/1) — editor_reply.md 참조

- [x] 3. Table 4 저작권: 자체 데이터 = 허가 불요 + Table 1 캡션에 자체작성 명시
- [x] 4. GenAI: §5.1 Methods 공시 문단 + Ack 제품상세 (Anthropic PBC 등)
- [x] 2. Refs 50건 전수 검증·DOI 부여 완료 (kim2025isis 서지 오류 발견·교체 포함)
- [x] 5. ICTC = 미결정(유저 확인) → "under review" 회신 문구 확정; 채택 시 proof에서 각주+서지
- [ ] 1. COI 폼: ICMJE 폼+가이드 유저 전달 완료 — 5인 서명 대기 (Kuc 실제 관계 확인 포함)
- [ ] 유저 확인 2건: kim2025isis 상세(Galvis Giraldo 풀네임 등), VDA5050 버전(2.0.0 vs 2.1.0)

## 블라인드 2차 주석 결과 (2026-09-04, 라벨러 = 김병준)

- 41/41 라벨 (한국어 자유어휘, 신뢰도 1–5). owner GT 대비: **채점 33건 중 31 일치 (93.9%)**, 불일치 2건은 저신뢰(2–3), 기권 5, 양측 불확실 2, owner 무응답 1
- 앵커링 교차표: owner가 모델 뒤집은 31건 → 블라인드 **25/26 재현** (기권 5); owner가 모델 수용한 9건 → 6/7 — 앵커링 징후 없음
- 반영: §5.1 GT 문단 + R1.7 + R3 응답 (PENDING 해소). 산출물: outputs/blind_kit/blind_vs_owner_scored.json, supplementary/blind_annotation/ (키트·답안·채점·매핑)
- [x] 유저 확인 완료 (9/4): owner GT 작성자 ≠ 김병준, 김병준은 T3 파이프라인/VLM 라벨·owner 답안 사전 열람 없음 → 원고 문구 확정

## 크로스 플랫폼 결정론 최종 결과 (2026-09-02 노트북 런)

- i9-13900H (Ubuntu 22.04 호스트, 커널 6.8) vs Ryzen 9 9900X (커널 7.0) — 동일 컨테이너
- 81 클러스터/244,975 pts/aggregate 9b36600d... **완전 일치**; Stage A 36s
- 증거 파일: revision1/determinism_report_i9laptop.json (+ blk360_seg determinism_kit/report_i9_13900H_laptop.json)
- 원고 §5.2·R3 응답서 반영 완료. R3.2 종결.

## 최종 번들 (2026-09-07) — 제출 준비 완료
- Zenodo DOI **10.5281/zenodo.22590752** (예약; 파일 5개+체크섬 업로드 후 Publish 필요) → 원고 Data Availability·R3 응답 반영
- 저자 프런트매터를 v28 제출본 기준으로 동기화 (Kuc ^{1,2}, 이메일 묶음) — 리포 main_mdpi.tex가 v28보다 뒤처져 있었음
- 산출물: ~/Downloads/tls_smf_electronics_overleaf_20260907_r1.zip (플랫, 42pp 클린), tls_smf_electronics_r1_20260907_preview.pdf, electronics_r1_responses_20260907/ (응답서 PDF 3부 + md + 에디터 회신 md), revision1/cover_letter_r1.md
- md→PDF 변환기: revision1/build/md2tex.py (pandoc 없음)

## 유저 결정 필요

- [ ] 리비전 마감일 (통보 메일 확인)
- [ ] 1.4 Uni3D ablation VLM 지출 승인 (~$3–4)
- [ ] 3.1 raw E57 공개 여부 (Zenodo 등)
- [ ] 1.7/3.4e 블라인드 재주석 — 정말 불가한지 (불가면 한계 서술로)
- [ ] 3.2 교차 머신 결정론 체크 — 가용한 두 번째 머신 있는지

## 제3자 제출 전 검토 반영 (2026-09-07 오후) → r1b
- 🔴 7건 전부 수정: ① threshold provenance 2분법(§5.1 Controls: base 4.1–4.6=T1/T2, corrective 4.7=T3 audit 포함 개발 후 cafe 전 freeze; R3 응답 동기화) ② Abstract/§8 identity 주장 → "zero observed identity switches on strong reference pairs + fragmentation false-new/absent 명시" ③ zero-cost → Abstract "controlled same-quality combined-edit re-scan 3.1%", §4.6/§7/§8 "identical-input repeat" ④ §4.7 layer4 wall-hug 정의에 0.15 m 추가 ⑤ panel square-up을 §4.7 layer2 finishing pass로 정의(6cm/5cm 마진; diffuse_resplit_v2._square_up_panels) + tab:layers 행 연결; Point-SAM "adopted"→"optional diagnostic, not in frozen pipeline" ⑥ color-variation: §5.8에 "render-source는 appearance-quality 컨트롤이지 pure photometric 아님, color-only 미실시" 명시 + R3 응답 정직화 ⑦ §5.4 "fully independent"→"distinct T3 owner-audited"; 블라인드 결론 완화(residual anchoring 배제 불가) + co-author 명시
- §4 도입부의 stale 문장("corrective mechanisms are introduced in Sec 5") → §4.7 참조로 수정 (검토자가 못 잡은 추가 모순)
- 응답서 톤 순화(R1 page 번호 제거·"summarized", R2 "outran"/"defensively"/"hastily" 제거, R3 파일 수 표현)
- 산출물: ~/Downloads/electronics_review/ 갱신 (overleaf_20260907_r1b.zip 43pp, preview r1b, 응답서 PDF 3부 재생성)

## 유저 그림 검토 반영 (2026-09-07 저녁) → r1c
- Fig1/Fig3 상단 여백: PNG 여백(20-30px)이 아니라 float 페이지 수직 중앙정렬 문제 → main/mdpi 프리앰블에 \@fptop=0pt, \@fpbot=0pt plus 1fil (float 페이지 상단 고정). 확인 완료.
- Fig12(fig6): 라벨 겹침 → make_paper_figs.py fig6에 씬별 오프셋 ("4-view" 위/아래 분리, showroom esc 라벨 endpoint 아래, robot hall esc 라벨은 선분 중점 아래), margins 확대, legend upper-left. 재생성·복사.
- Fig16 메가클러스터(machine_012): 그대로 둠 — §7 coverage paradox의 실증 근거이자 rev3 전환 실측(9/1/31/45)과 정합; det4 재분할본으로 바꾸면 본문 수치가 깨짐. 리뷰어 미지적.
- 산출물: electronics_review/ → overleaf_20260907_r1c.zip + r1c preview
- (r1d) 유저 재지적: float 페이지 하단 공백 → \floatpagefraction 0.85 / topfraction 0.9 / bottomfraction 0.6 / textfraction 0.05 / topnumber 3 (main+mdpi 프리앰블). Fig1·Fig3 아래로 본문이 이어짐 확인, 42pp. electronics_review/ = overleaf_20260907_r1d.zip + r1d preview.
- (r1e) 제3자 2차 피드백: 응답서 3건(R3 provenance·color·co-author, R1 page)은 r1b에서 이미 수정돼 있었음(검토자가 구버전 열람). 원고 잔여 2건 수정: §2 Point-SAM "appears in our pipeline"→"evaluated only as optional diagnostic; not part of frozen pipeline", §8/§7 cost 문구 "can scale with detected content change under reproducible acquisition". Overfull: tab:query·tab:backbones footnotesize+tabcolsep, formal summary tier set을 2줄 display로, Controls texttt에 \hspace{0pt} → 잔여 overfull ≤8.4pt. electronics_review/ = r1e (42pp).
- (r1f) 제3자 3차: §1 기여1 cost 문구('proportional to actual change'→'under reproducible acquisition… track detected content change'), §4.7 'scene-independent thereafter'→'no per-scene adjustment at execution; held-out transfer only where reported in 5.7'. electronics_review/ = r1f.
- (r1g) 제3자 4차(응답서 톤 3건): R2 TOSM 78-81% accuracy vs 85% recall → contextual/not like-for-like (§1 본문도 동일 수정), R1/§5.1 'decisive'→'most relevant/informative comparison', R3 benchmark 'did not intend'→'original wording could have suggested… we agree'. electronics_review/ = r1g.

- **2026-09-09 — 제출 완료.** r1g 세트를 susy에 업로드(원고 zip + preview PDF, 응답서 3부, cover letter, non_published_material.zip; Open Peer Review = No; GenAI 신고 체크) 및 Ms. Mina Yan에게 editor_reply + 서명 CoI 폼 회신. 이후 편집부 회신 대기.
