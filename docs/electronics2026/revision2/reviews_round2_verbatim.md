# electronics-4525706 — Round 2 reviews (received 2026-09-14, revise within 5 days → ~2026-09-19)

Reviewer 1: all clear. Reviewer 2: no response.

## Reviewer 3 (review dated 07 Sep 2026)
English: fine. Intro/background: Yes. Research design: Yes. Methods / Results / Conclusions / Figures: **Can be improved**.

1. Independent building and facility validation has not been performed; the authors have identified this as future work. The revised article lacks full end-to-end validation involving different scanning conditions, different building geometry, and an independent facility.
2. There is no end-to-end task validation on a real robot.
3. The requirement for independent ground truth has not been fully met.

## Editor checklist (email)
(I) references relevant; (II) highlight revisions; (III) cover letter with point-by-point response; (IV) critically assess recommended refs; (V) explain anything impossible to address.

## Revision 2 status (2026-09-15) — r2a bundle ready
- New §6.3 Real-Robot Mission Execution + Table 14 + Fig. 20 (figs/fig20_real_robot_missions.png) + Video S1 (\supplementary in main_mdpi.tex); abstract/§1/§6 intro/Fig.19 caption/§7 Scope/§8 updated. Platform "2nd-goal" failures NOT reported in the manuscript per user decision (missions issued one at a time from a standing start).
- R3 point 1 answered by scoping (cafeteria §5.8 + real-robot independent map); point 3 by three evidence layers (owner audit, blind co-author annotation §5.1, membership-independent sweep reference §5.2); no external annotator, stated plainly.
- Build: revision2/build/overleaf_r2 (flat, 44 pp, 0 errors/undefined), marked = build/marked_r2/main_diff.pdf (latexdiff --flatten -t CFONT vs r1g; 236 adds/16 dels), response/cover via revision1/build/md2tex.py.
- Bundle: ~/Downloads/electronics_review2/ (overleaf zip r2a, preview, marked, response_reviewer3.pdf/md, cover_letter_r2, VideoS1 mp4, fig20, missions CSV).

## 2026-09-15 — 제출 완료 (r2b: 외부 블라인드 라벨 반영본). 편집부 결정 대기.
