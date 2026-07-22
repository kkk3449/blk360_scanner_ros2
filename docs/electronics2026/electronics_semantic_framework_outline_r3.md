# Outline r3 — Electronics (Special Issue: Intelligent Perception and Control for Robotics, 2nd Edition)

**Title:** From Terrestrial Laser Scans to Queryable Robot Knowledge: A VLM-Verified Framework for Incremental 3D Semantic Modeling
*(선배 Electronics 평가서 §10 추천안; incremental을 제목에 노출)*

**Authors:** Sangmin Kim (1,2), [Haryeong Kim (1,2) — pending 5.2 outcome / contribution split], Tae-Yong Kuc (1,2,\*)
(1) Dept. of Electrical and Computer Engineering, Sungkyunkwan University; (2) CASELAB Co., Ltd. (\* Corresponding author)

**Type:** Research Article (extension of ICTC 2026; ISIS 2025 cited for KG lineage; both declared)
**Target length:** 20–24 pages, ~10 figures, 6–8 tables
**Submission target:** late July – early August 2026

**r3 changes vs. r2 (선배 Electronics 평가 반영):**
- Contribution 4개 → **3개 압축**; 벤치마크는 contribution이 아니라 validation으로 격하.
- **Stable ID 재서술**: "spatial anchor hash = ID"가 아니라 **mint-once immutable ID + matching 기반 identity 유지** (구현은 이미 이 구조; anchor hash는 최초 발급 시의 fingerprint로만 서술).
- **Determinism 주장 범위 제한**: deterministic geometric backbone + confidence-controlled semantic inference. `byte-identical`은 same-host 실측치로 보고하고 tolerance 기반 정의 병기.
- **"Fully automated" 조건 명시**: automated *after registered TLS input*.
- Methodology에서 **4.5 Object identity matching을 4.6 upsert에서 분리** (평가서 §9).
- 쿼리 벤치 **4조건** (LLM-only / ungated / verified-only / gated=verified+tagged-unverified) + atomic-claim hallucination 정의 + 통제조건 명문화.
- 5.5를 **시나리오 A–D + identity 지표**로 확장; **5.6 Ablation & sensitivity 신설** (2.5 m rule 등).
- 신규 방법 요소 반영: **구조물 잔재 필터**(천장 soffit), **구조적 게이팅**(지시문 게이트 실패 실측 → 인터페이스 강등).
- Cognee: 미구현이므로 본문에서 제거 (demonstration에서도 Neo4j만).

---

## Abstract (~250 words, 평가서 §11 7문장 골격)
1. *Problem:* indoor robots need semantic environment representations that are queryable, **reliable, and maintainable** after changes.
2. *Limitations:* open-vocabulary pipelines propagate unverified labels; build-once scene representations lack a principled update path.
3. *Framework:* deterministic geometric object decomposition (+ structure-remnant filtering) / multi-view VLM verification with automatic escalation / confidence-gated graph upsert.
4. *Incremental:* identity-preserving matching → object-level insert/absent/update without graph rebuild.
5. *Evaluation:* two indoor TLS scenes; segmentation repeatability, verification accuracy, escalation cost, query hallucination, update correctness.
6. *Headline numbers (확보분):* escalation lifts accuracy 75.0→86.1 % (showroom) and 85.3→91.2 % (vis_n2); unanimous-vote precision 100 %; gated KG answers 85.4 % vs. 50.0 % LLM-only with hallucination 20.8→14.6 %; incremental re-verification costs 2.7 % of a full rebuild (0 % on repeat).
7. *Significance:* confidence-aware verification + incremental maintenance make TLS-derived semantic knowledge usable for robots.

## 1. Introduction
- Queryable / fact-consistent / **maintainable** environment knowledge 필요.
- Gap 1: open-vocabulary 3D classifiers degrade on industrial scenes (PointCLIP V2 collapse; PTv3/SPFormer transfer failure; **자체 vocabulary ablation: 어휘를 좁혀도 정확도 하락 66.7→33.3 %** — embedding이 병목).
- Gap 2: ungrounded LLM querying hallucinate (ISIS finding + 자체 벤치 20.8 %).
- Gap 3: 기존 파이프라인 semi-automated + build-once.
- **Contributions (3):**
  1. **Deterministic geometric & explicit-modeling backbone** — registered TLS 입력 후 완전 자동; seeded-RANSAC plane removal, fixed-parameter DBSCAN(+giant-split), 구조물 잔재 필터, PCA explicit modeling; same-host 재실행 byte-identical (재현성·diff 안정성의 기반).
  2. **Confidence-aware multi-view VLM verification with automatic escalation** — 4-view forced-schema 분류 → confidence-weighted late fusion; split vote → 8-view/zoom 자동 escalation; 미해결은 숨기지 않고 `unverified`로 유지 (human review 없음).
  3. **Identity-preserving incremental KG update** — mint-once ID + 2-pass matching(공간→타입·치수), unchanged/updated/moved/inserted/absent 분류, confidence-gated upsert; graph rebuild 없이 node-level 유지보수.
- Validation: labeling/verification, gated querying(4조건), incremental update(시나리오 A–D) 벤치마크.

## 2. Related Work
- 2.1 3D semantic mapping & scene graphs (3DSG, Open3DSG, MoMa-LLM; TOSM lineage: SMF → **DK-SMF** (Joo et al., Electronics 2025) — RGB-D 탐색 기반 object/place 자동 모델링; 본 논문은 TLS 백본+검증 게이팅+증분 갱신으로 확장, 차별점 표로 명시).
- 2.2 Open-vocabulary 3D recognition and limits (Uni3D, PointCLIP V2; 자체 domain-gap/vocabulary 증거).
- 2.3 Instance segmentation: learned (SPFormer, PTv3, Point-SAM) vs. geometric — 포지셔닝: **novel algorithm이 아니라 reproducible decomposition** (평가서 §7.4 표현 채택).
- 2.4 Multi-view VLM verification (ICTC 2026 확장 지점 명시).
- 2.5 Knowledge-grounded querying & hallucination mitigation.
- **2.6 Incremental semantic-map maintenance** (별도 subsection; scan-once-build-once 대비 차별화 축).
- 2.7 (short) 2D map vs. 3D map vs. 3D semantic KG capability matrix → **Table 1** (분석만, 실험 없음).

## 3. System Overview
- Fig. 2: five-stage architecture + escalation loop + update loop.
- **Deterministic vs. stochastic 구분을 아키텍처 그림에 명시** (평가서 §7.2): geometric backbone(결정적) / semantic inference(확률적, voting·gating으로 통제).
- I/O contract per stage; verified/unverified data flow; TOSM 3-layer (symbolic/explicit/implicit).
- Automation 범위: **"fully automated after registered TLS input"** 문구 고정.

## 4. Methodology
### 4.1 TLS Preprocessing
Registered E57 → voxel downsample → **seeded-RANSAC** iterative plane removal (floor/walls/ceilings). 모든 난수 시드 고정 → same-host byte-identical 재실행 (실측; 이식성은 tolerance 기반 "numerically unchanged" 정의 병기).

### 4.2 Geometric Object Decomposition + Structure-Remnant Filtering
- Fixed-parameter DBSCAN (eps 0.30 / min 100) + conservative giant-split (footprint>2.5 m → eps 0.20 재클러스터).
- **구조물 잔재 필터 (신규, 실환경 발견 기반):** 다층 천장(레벨 3개: z 2.48/2.18/1.95)의 높이 전환부 수직 soffit 띠가 phantom 객체로 남는 문제 → "얇은 수직 평면띠 + 상부 최상단 조밀 수평층(=진짜 천장)" 규칙으로 제거 (showroom 45→38, vis_n2 88→81; 유지 클러스터 byte-identical·ID 불변). 오탐 방지 조건(창턱/벽 잔재 구분) 포함.
- 포지셔닝: reproducible decomposition **designed for verification and incremental diffing** (novelty 주장 아님).

### 4.3 Explicit Modeling and Provisional Classification
BBox centroid, PCA yaw, extents, color (결정적); Uni3D provisional label (확률적, 후단 검증 대상임을 명시).

### 4.4 Multi-View VLM Verification with Automatic Escalation
4-view → per-view forced-tool-use 분류 → confidence-weighted fusion (viewVotes/voteShare 기록) → split vote 시 8-image escalation (대각 4 + zoom 4) → share≥0.6 `verified_escalated`, else `unverified`. Implicit attributes at fusion. Prompt/decoding/model version 기록 (평가서 체크리스트).

### 4.5 Object Identity Matching (분리된 절)
- **Identity 정책: mint-once.** 최초 등장 시 ID 발급(birth-anchor fingerprint에서 유도; 이후 불변).
- 재스캔 매칭: 1-pass 공간(centroid 0.5 m + extent 30 %/0.15 m) → 2-pass 이동 탐지(type+dims 일치, ≤6 m) → unmatched 처리(신규 mint / absent 마킹). Hash는 ID가 아니라 **candidate retrieval fingerprint**.
- 평가 지표(→5.5): matching accuracy, identity-switch rate, false-new, false-absent, attribute-update accuracy.

### 4.6 Confidence-Gated Graph Upsert
unchanged/updated/moved/inserted/absent 분류; per-node history/provenance; 상태별 gating(verified/verified_escalated/unverified); mapId-scoped Neo4j Cypher.

### 4.7 Query Interface with Structural Gating
- **핵심 발견을 방법으로 승격:** 지시문("unverified를 주장하지 말라")만으로는 LLM이 게이트를 안 지킴(트랩 1/4) → **구조적 게이팅**: query-time view에서 unverified 노드의 type을 `unknown`으로 강등하고 `failedCandidateType`으로 분리 노출 → 트랩 4/4. "Gate by construction, not exhortation."

## 5. Experiments
### 5.1 Setup
- Scenes: showroom (industrial; det 38 objects + ICTC 57-set은 5.3 비교용 archived artifact로만) / vis_n2 (robot hall, det 81 objects). Table 2: scene stats.
- GT: 렌더 감사 + 소유자 확인 2-pass (showroom: confirmed 36/plausible 12; vis_n2: confirmed 34/plausible 18; unknown은 평가 제외, 구조물 잔재는 별도 분류).
- Metrics 정의 선행: top-1 vs GT(synonym 관대 매칭 명시), unanimity/escalation precision, **hallucination = unsupported atomic claims / all factual atomic claims**, update 지표(4.5 목록).
- 통제조건 (4조건 공통): same LLM(claude-sonnet-4-6)/system prompt/query set/decoding/retry/answer schema(forced tool use)/정보 접근 범위 문서화. LLM-only 입력 = 동일 씬의 Stage-A object dump (명시).

### 5.2 Segmentation: Repeatability, Filtering, Sensitivity
- Repeatability: E57→objects 2-run byte-identical (양 씬, 10 planes 포함) ✅확보
- Structure-remnant filter 검증: 소유자 GT로 7/7 phantom 제거 확인, 유지 클러스터 무변화 ✅확보
- **Sensitivity (신규 실험, 평가서 §7.5):** footprint {1.5, 2.0, 2.5, 3.0 m} × eps 3값 × min_points 3값 (+voxel 2값) → merge/split error, instance P/R, 최종 라벨 정확도, runtime □예정 (결정론적·무료)
- Point-SAM pilot (사전등록 채택룰): 분리 성공 3/6, 과분할 2 — 채택룰 최종판정 □예정; 미채택 시 negative-result 단락으로 서술.

### 5.3 Verification: Fusion and Escalation
- ✅확보: showroom base 75.0→esc 86.1 % (confirmed n=36; 회복 5/퇴행 1; trigger 58 %); vis_n2 base 85.3→esc 91.2 % (n=34; 회복 3/퇴행 0; trigger 52 %).
- ✅Unanimity precision 100 % (21/21, 36/36); verified-등급(만장일치+escalated) precision 100 %/100 %; unverified 50 %/40 % → gating 근거.
- Escalation accuracy–cost curve (호출수 232/312, ~$0.009/call) ✅데이터 확보, 그림 □예정
- **Error taxonomy** (평가서 Priority 2): (a) 구조 잔재 유발 phantom (soffit→keyboard; 필터로 해결), (b) 일관 오독-escalation 통과 (vis_n2 id83 매달린 체인→door_handle 만장일치; id75 keyboard escalated→소유자 반박), (c) 렌더 모호 → unverified. "만장일치≠정답" 사례를 verifier-bounded gate 논거로.
- ICTC 4-view-only와 비교 (extension 근거).

### 5.4 Knowledge-Grounded Query Evaluation (4 conditions)
- 조건: ① LLM-only ② ungated KG ③ **verified-only KG** (신규) ④ gated KG (verified + 구조 강등된 unverified). 평가서 §7.6 4조건 채택.
- ✅확보 (showroom, ①②④): LLM-only 50.0 %/halluc 20.8 %/기권 14 — ungated 81.2 %/18.8 %/트랩 0/4 — gated **85.4 %/14.6 %/트랩 4/4**.
- ✅**지시문 게이트 실패 실측** (구 gated=ungated) → 구조적 게이팅 도입 서사; ablation으로 보고 (5.6).
- □예정: showroom ③ 추가 재실행, vis_n2 4조건 (~$3).
- Per-type breakdown (symbolic/explicit/implicit/scene/trap); trap 선정 원리(unverified-only 라벨) 명시.

### 5.5 Incremental Update Evaluation (Scenarios A–D)
- **A: Unchanged repeat re-scan** — unchanged-node stability; ✅부분확보 (재실행 시 81/81 md5 carry, VLM 0 calls) □형식화 예정
- **B: Moved objects** — identity 유지 + pose update; ✅부분확보 (chair 이동 → moved 1/1, ID 불변) □단독 시나리오 예정
- **C: Removed + added** — absent/insert; ✅부분확보 (toolbox 제거→absent, 복제 삽입→insert, 삽입본 라벨 독립 재검증 일치) □단독 예정
- **D: Partial visibility (occlusion)** — false-absent 방지 평가 □신규 예정 (클러스터 부분 크롭 합성)
- 지표: matching accuracy, new/absent accuracy, **identity-switch rate, false-new, false-absent**, unchanged stability, pose/extent update error, **selective re-verification cost (실측 2.7 % of full; repeat 0 %)**, runtime vs full rebuild, post-update query 일관성.
- kg_upsert에 지표 자동 산출 추가 □예정 (소규모 코드).

### 5.6 Ablation and Sensitivity (신설)
- Escalation on/off (=base-4 counterfactual) ✅
- **Gating 방식 ablation: note-only vs structural** ✅ (v4_notegate vs v4)
- **Pose-context(설치높이) ablation** ✅: 천장 객체 표적 방어로는 결정적(phantom keyboard 3개 제거), 전면 적용은 무익(런투런 분산 수준) — targeted defense로 서술
- Uni3D vocabulary ablation ✅ (generic 30 66.7 % > curated 20 48.7 % > owner 18 33.3 %; run-to-run 분산 5 %p)
- Segmentation parameter sensitivity □예정 (5.2와 공유)

### 4.8 Object-Grounded Place Layer (보조 절, DK-SMF 계보)
DK-SMF place 스키마(boundary polygon/isInsideOf) 승계: 수동 섹션 폴리곤
(TIPS 2025) 정합 import + point-in-polygon isInsideOf 자동 산출 +
**ring code**(구역 중점 방위각 순 verified 객체) → LLM place 명명 —
"place 시맨틱은 검증 통과 객체에서만 유도"(게이트 전파). 2-시점 명명 진화
실측: T1 control_panel_station/machine_workspace → T2 storage/seating
(전시→테스트베드 개편 반영). SLIC 자동 room segmentation은 DK-SMF 인용,
robot 층은 future work.

## 6. Application Demonstrations (contribution 아님)
- 6.1 Knowledge-grounded 대화 예시 (confidence 노출 응답).
- 6.2 Isaac Sim semantic injection (트윈 구축은 RAAICON 인용, semantic layer만 여기서).
- 6.3 Manipulation-readiness note (statement only).
- (Cognee 제거; Neo4j 시각화는 그림으로만.)

## 7. Discussion
- VLM cost/latency (escalation은 trigger rate로 bounded; 실측 $/scene).
- Residual unverified (숨기지 않고 노출; gate 품질은 verifier 품질에 종속 — 5.3 taxonomy 사례).
- **구조물 잔재의 잔여 한계**: 수직 soffit는 필터링, 수평/경사 천장 패치·개구부 너머 콘텐츠는 잔존 (vis_n2 6건; 향후 확장).
- Single-scanner, object-level 변화 한정 (structural change future work), 씬 2개 일반화 한계.

## 8. Conclusion
Recap; future work: structural-change updating, horizontal/sloped remnant filtering, embodied task validation, temporal dynamics, multi-robot shared KB.

---

## Figure plan (평가서 §14 우선순위 반영)
1. Overall framework architecture (deterministic/stochastic 색 구분, escalation+update loop) [필수]
2. Geometric pipeline + structure-remnant filter inset (soffit 사례 before/after) [필수]
3. Multi-view verification & escalation flow + viewVotes 예시 [필수]
4. Verified/unverified object record 예시 (JSON 카드) [필수]
5. Identity matching & upsert 상태 다이어그램 (before/after re-scan) [필수]
6. Escalation accuracy–cost curve (양 씬) [필수]
7. Query benchmark 4조건 결과 (막대 + 트랩 분리) [필수]
8. Incremental update before/after (씬 렌더 + diff 색) [필수]
9. Isaac Sim semantic twin + Neo4j 그래프 [선택]
10. Error-case taxonomy 패널 (soffit/체인/keyboard 렌더) [선택 — 리뷰 방어력 높음, 권장]

## Table plan (평가서 §15)
1. 2D vs 3D vs 3D-KG capability matrix
2. Scene statistics (det set, 필터 전/후)
3. Segmentation repeatability + sensitivity 요약
4. Verification & escalation accuracy (양 씬, ICTC 대비)
5. Query benchmark 4조건 (main table)
6. Incremental update 시나리오 A–D 지표
7. Node/edge schema
8. (선택) runtime + VLM cost

## Reviewer 예상 코멘트 대응 (평가서 §12 내장)
- "기존 기술 조합" → verification–gating–update 통합 설계가 기여; 5.6 ablation으로 모듈별 효과 분리.
- "DBSCAN 표준기법" → novelty 주장 안 함; reproducible backbone 포지셔닝 (4.2).
- "이동하면 ID 바뀜" → mint-once + matching; identity-switch rate 정량 (5.5).
- "VLM 확률적" → determinism 범위 제한 + voting/gating/fixed decoding 명시.
- "벤치 불공정" → 4조건 동일 통제 + atomic-claim 정의 + LLM-only 입력 명시.
- "update가 demo 수준" → 시나리오 A–D + 반복 + 지표 (5.5).
- "범위 과다" → 3 contribution + demonstration 분리.
- "conference 중복" → extension disclosure: ICTC(4-view 분류 기반) / ISIS(KG lineage) 대비 신규 = escalation, 구조적 gating, identity matching, incremental update, 4조건 벤치, 2-scene, remnant filter, error taxonomy.

## Writing checklist (carry-over + 평가서 §16)
- Conference extension 선언; 중복 figure 재사용 시 citation/disclosure.
- Determinism·automation 표현: "deterministic geometric backbone + confidence-controlled semantic inference", "fully automated after registered TLS input".
- `byte-identical` → same-host 실측 + tolerance 정의 병기.
- VLM prompt/decoding/모델 버전 기록; GenAI usage disclosure; TIPS/CASELAB COI.
- Data/code availability 문구; 추천 리뷰어 후보; APC 할인/바우처 확인.
- Haryeong Kim authorship은 5.2 판정 후 확정.

## 실험 상태 요약 (drafting 기준, 2026-07-21 저녁 갱신)
| 항목 | 상태 |
|---|---|
| 결정론 2-run, remnant filter, 5.3 양씬 (showroom 75.0→86.1 n=36 / vis_n2 78.1→84.4 n=32, 룸스코프), 게이팅·pose-context·vocab ablation | ✅ |
| 파라미터 sensitivity 36조합×2씬 (footprint 2.0–2.5 안정, 3.0 병합·1.5 과분할) | ✅ |
| 4조건 벤치 양씬 — showroom: llm 52.1 / ungated 81.2 / verified-only 58.3(트랩 4/4, 기권 12) / **gated 83.3, halluc 16.7, 트랩 3/4**; vis_n2: 50.0/56.2/46.9/59.4 (냉장고 트랩 게이트 성공, escalated-keyboard는 양 조건 실패 = verifier-bounded 2씬 재현) | ✅ |
| 5.5 시나리오 A–D — A 55/55·VLM0·switch0 / B moved 1/1 ID유지 / C absent1+insert1 / D2(20%) 2/2 updated / D(40%) 매칭실패 switch2 → 한계 경계 정량화 | ✅ |
| identity 지표 kg_upsert 내장(선택), Point-SAM 채택판정, 그림 생성(Fig 1–10), 본문 집필 | □ 남음 |
