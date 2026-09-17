# Assistant-editor 요청 5건 (Ms. Mina Yan, 2026-09-01 수신) — 대응 현황

원문은 유저 메일 참조. 회신 초안은 아래 [REPLY DRAFT] — ICTC 상태·COI 서명 확인 후 발송.

## 1. COI form (회사 소속 저자) — **유저 액션**
- 대상: S.K./Y.S./B.K. (CASELAB Co., Ltd. 고용, 논문 COI 문구에 이미 명시). Kuc은 affiliation만.
- MDPI COI 폼: susy 제출 페이지 또는 editorial 안내 링크에서 다운로드 → 해당 저자 서명 → 회신 첨부.
- 폼 기재 내용은 원고의 \conflictsofinterest 문구와 일치시킬 것: "employed by CASELAB Co., Ltd."; 그 외 이해관계 없음.

## 2. Refs 검증 — **완료** (refs_verification.md, refs.bib 반영, main 40pp/mdpi 42pp 클린)
- 50건 전수: 38 OK / 10 정정 / DOI 38건 + 안정 URL 7건 부여, 발명 DOI 0.
- 플래그 8건: vda5050→**v2.0.0 (2022), VDA/VDMA + 공식 URL**; ester1996→AAAI URL(DOI 무);
  kim2026ictc→"submitted (under review)" 노트(ICTC 미결정, 유저 확인됨);
  lewis2020rag→NeurIPS 33 공식 URL; **kim2025isis는 제목·저자가 틀려 있었음** →
  실제 서지로 교체 (H. Kim, K. Joo, G. Galvis Giraldo, S. Kim, T.-Y. Kuc,
  "Toward Long-Term Memory in Embodied AI: ...", Proc. 26th ISIS, Cheongju, 2025,
  pp. 237–240) — **온라인 미색인: Galvis Giraldo 풀네임+최종 프로시딩 정보 유저 확인 필요**;
  deng2026ovimap→**CVPR 근거 없음, arXiv 재인용**(10.48550/arXiv.2603.26541; 본문 프로즈는
  CVPR 명명 안 해서 무수정); zhai2026onlinepg→CVPR26 유지+arXiv DOI;
  raaicon2026→"accepted; to appear" + Jashore 주소.
- 추가 정정: mask3d/openscene/pointnet++/openmask3d 페이지 보완, funcgraph CVPR25 DOI.
- 잔여 유저 확인 2건: ① kim2025isis 상세 ② VDA5050 구현 기준 버전이 2.0.0 맞는지(현행 2.1.0).

## 3. Table 4 저작권 — **대응 완료 (허가 불필요)**
- v28 Table 4 = tab:backbones = 우리 실험 수치 (인용은 비교 대상 시스템 명명뿐) → 각색 아님.
- 만약 에디터 의도가 Table 1(tab:dksmf)이어도: 저자 자체 작성 비교표, DK-SMF의 표/그림 재사용 없음 — 캡션에 "authors' own tabulation; reproduces none of its tables or figures" 명시 추가함.

## 4. GenAI 공시 — **대응 완료**
- §5.1에 "Generative AI use." 문단 신설: (i) 실험 구성요소로서의 VLM/LLM (모델·프롬프트·스키마·usage 아카이브) vs (ii) 집필 보조 (언어 편집·LaTeX) 역할 분리 명시.
- Acknowledgments에 제품 상세 보강: Claude, model claude-sonnet-4-6, Anthropic PBC, San Francisco, CA, USA, https://claude.ai.
- 제출 시스템의 GenAI 선언 체크박스도 재제출 때 체크 (유저).

## 5. 학회논문 확장 요건 — **부분 완료, ICTC 상태 필요**
- (1) 분량: 40쪽 저널 논문 — 충족. (2) 인용+1면 각주: ref [30] 인용됨; \conference 각주는 ICTC 미결정 상태라 주석 처리돼 있음 → **ICTC 채택 확인 시 실제 제목("Multi-View VLM Verification for Semantic Object Extraction from a Terrestrial Laser Scan Toward TOSM-Based Robot Environment Models")으로 활성화** (주석의 옛 가제 쓰지 말 것); 미결정이면 "under review, cited as such" 회신. (3) 저작권: ICTC 미게재 상태면 저자 보유 → 허가 불요; IEEE 저작권 양도는 camera-ready 시점 발생 (그 경우에도 확장판은 재사용 아닌 확장이라 IEEE 정책상 인용+주석으로 충분 — 필요 시 명시). (4) 커버레터: 확장 사실 + 변경 요약 문단 (기존 cover_letter_electronics.md의 ICTC 문단 재사용+갱신).

## [REPLY DRAFT] (영문, 확인 후 발송)

Dear Ms. Yan,

Thank you for the careful checks. Point by point:

1. (COI form) Authors Sangmin Kim, Yonghyeon Song, and Byeongjun Kim are employed by CASELAB Co., Ltd., as disclosed in the manuscript's Conflicts of Interest statement. The signed COI form(s) are attached.

2. (References) We have verified all 50 references against Crossref/DBLP/publisher records and added a DOI or stable link to every entry in the revised manuscript. Regarding the flagged items: ref. 16 is the VDA 5050 industry specification (now cited with version and official URL); ref. 29 (Ester et al., KDD-96) has no DOI — the stable AAAI proceedings link is provided; refs. 30 and 50 are the authors' own conference papers ([30] currently under review at ICTC 2026, and [50] accepted for presentation at IEEE RAAICON 2026, to appear in the proceedings), cited with explicit status notes as no DOI exists yet — we will update both citations at the proof stage if their status changes; ref. 33's proceedings details have been completed; refs. 36–37 are now cited by their arXiv records with DOIs. A full verification table is included with the revision.

3. (Table 4) Table 4 presents the authors' own experimental measurements on our scan data; it reproduces or adapts no published table or figure — the cited works are the systems being compared, and all numbers are ours. No permission is required. (Table 1, likewise, is the authors' own tabulation positioning our work against DK-SMF; its caption now states this explicitly.)

4. (GenAI) The revised manuscript adds a dedicated disclosure paragraph in the Materials-and-Methods part (Section 5.1, "Generative AI use") separating the two roles of generative models in this work — as experimental components under test (fully documented, contributing no text) and as a writing aid for language editing — and the Acknowledgments now carry the product details (Claude, model claude-sonnet-4-6; Anthropic PBC, San Francisco, CA, USA).

5. (Conference expansion) This article expands our conference paper [30] well beyond the article threshold, and the cover letter discloses the expansion with a summary of what has been added. One clarification on status: the conference paper is currently under review at ICTC 2026 (submitted July 2026) and is cited with that status; it has not been published, so the copyright remains with the authors and no permission is required. Should it be accepted and published before this article's publication, we will add the standard first-page extended-version note and update the citation at the proof stage — please advise if you prefer a different handling for a conference paper that is still under review.

Kind regards,
[corresponding author]
