# Response to Reviewer 2 — sensors-4464625 (Major Revision)

**Manuscript:** Occlusion-Aware Visibility Coverage for Robotic Stop-and-Scan 3D LiDAR Mapping

We thank the reviewer for the constructive review. The most substantial change prompted by this review is a **new experiment**: a disk baseline governed by the *identical* accept/skip rule as the proposed policy (see Comments 2 and 5 below). Each comment is quoted before its response; all changes are visible in the marked-up manuscript.

---

## Comment 1 — Overall assessment

> *Overall, the paper is well presented...*

**Response.** We thank the reviewer for the positive overall assessment and address each of the specific comments below in turn.

## Comment 2 — Sufficiency and representativeness of the isotropic-disk baseline

> *It is not clear why the isotropic disk method alone is sufficient to evaluate the proposed approach. Is this baseline representative of the current state of the art? Why additional methods are not considered in the comparison? Additional comparisons, or a stronger justification for the selected baseline it is necessary to strengthen the experimental evaluation.*

**Response.** We have strengthened the evaluation in both of the ways the reviewer suggests.

*Additional comparison.* The revision adds a fourth policy to the controlled ablation: a **disk marginal-gain baseline** that applies the identical accept/skip rule (Equation (8)) as the proposed policy, with the disk coverage model B_disk substituted for the visibility region B. The four compared policies are now: (i) uniform no-skip (the fixed-interval policy of stop-and-go practice; upper reference), (ii) disk spacing (proximity rule), (iii) disk marginal-gain (matched rule; **new**), and (iv) ray-cast visibility (ours). See Sections 4.2 and 6.2, Table 2, and Figure 8. Notably, the matched-rule disk baseline reaches only **59.1 ± 1.3% LOS** in the multi-room world — worse than even the spacing rule — because the disk model's through-wall over-claiming saturates its own coverage bookkeeping (mechanism explained in Section 6.2).

*Justification of the baseline set.* Section 6's preamble now justifies this set explicitly: the disk/proximity treatment is a commonly used online coverage model in stop-and-go practice (Section 2), while published TLS scan-planning methods — including the strongest recent ones, now cited (Dehbi et al., ISPRS J. Photogramm. Remote Sens. 2021; Knechtel et al., ISPRS J. Photogramm. Remote Sens. 2025) — optimize viewpoints *offline over a prior environment model* (floor plan, BIM, or coarse scan) and are therefore not applicable as baselines in the online unknown-environment regime studied here, where every decision must be made on the live SLAM map. NBV planners for continuous sensors likewise do not decide whether to pay a large fixed stationary cost. The uniform no-skip reference bounds the coverage attainable along each path, so the reported gains are located within a bracketed range rather than against a single baseline.

## Comment 2 (continuation) — Availability of the conference manuscript (reference [9])

> *Since the present manuscript is explicitly described as an extension of the conference work, the authors should clearly identify the differences and the additional contributions of the present manuscript. Moreover, since reference 9 is not yet available, the conference manuscript should be made available to the editors and reviewers.*

**Response.** The Introduction now discloses the overlap item by item:

> "The conference version contributes the stop-and-scan system integration (frontier exploration coupled to a scan sequencer) and an isotropic-disk scan trigger, demonstrated in simulation. New to this article are: the ray-cast visibility coverage model and the LOS coverage metric (Section 3); the dual-criterion marginal-gain placement rule and the coverage-completion phase (Section 4); the controlled paired ablation with its three baselines and the parameter sensitivity analysis (Section 6); and the entire hardware study — the fully autonomous on-hardware disk-versus-visibility comparison and the per-run offline registration of the acquired scans (Sections 6.6 and 6.7). No figure, table, or quantitative result is shared between the two manuscripts."

The conference manuscript has been uploaded to the submission system for the editors and reviewers, as requested. Since the original submission, the conference paper has been **accepted for presentation at ICCAS 2026** (Regular Paper; decision of 31 July 2026), and its status has been updated accordingly in reference [9], in the Introduction, and in the first-page extended-version footnote.

## Comment 3 — Thresholds 25 and 65 in Equation (1)

> *For equation 1, it is not clear why the authors selected the thresholds of 25 and 65.*

**Response.** Section 3.1 now explains the choice: these are the long-standing free/occupied convention of the ROS occupancy-map pipeline (probability thresholds 0.25/0.65), and the classification is insensitive to their exact placement because SLAM-estimated occupancies are strongly bimodal — almost all mapped cells settle near 0 or near 100 — so any thresholds separating the two modes yield the same classification. The deliberately separated band maps ambiguous cells to *unknown*, which the visibility model treats conservatively: they block rays and are never counted as covered.

## Comment 4 — Does Equation (3) represent ideal geometric visibility?

> *For equation 3, it would be useful to clarify if the proposed equation represents the ideal geometric visibility, since the equation does not explicitly include K (the number of rays) or the angular resolution used in the presented approach.*

**Response.** Correct, and this is now stated explicitly after Equation (3): "the set definition above characterizes the ideal geometric visibility region and does not depend on the ray discretization; the K rays and half-cell marching are the implementation that approximates it. At the range bound the inter-ray arc spacing is R·Δθ ≈ 5.2 cm for R = 6 m, on the order of a single grid cell (ρ = 5 cm), so the discrete approximation recovers the ideal region up to single-cell effects at region boundaries."

## Comment 5 — Acceptance logic of the disk baseline; Table 2's claim

> *However, it is not clear if the same acceptance logic is applied to the isotropic disk baseline, since later it is described as skipping candidates located within R of an existing scan. Therefore, the statement "The only difference between rows is the coverage model used for the scan/skip decision" in table 2 should be reconsidered.*

**Response.** The reviewer is right, and this observation prompted the main new experiment of the revision. The spacing rule is *not* the marginal-gain rule applied to B_disk, and Section 4.2 now says so explicitly ("a candidate closer than R to an existing scan can still contribute a crescent of new disk area"). To make the comparison well-posed, we added the **disk marginal-gain baseline** described under Comment 2: the identical Equation (8) rule with B_disk substituted for B and disk-based bookkeeping of the covered set, replayed on the same fixed candidate paths. Table 2 now contains four rows per environment, and its caption has been corrected to:

> "...the disk marginal-gain row applies the identical accept/skip rule as the visibility policy, so within each environment that pair differs only in the coverage model."

The former claim ("the only difference between rows is the coverage model") has been removed. The outcome sharpens the paper's conclusion: under the identical acceptance rule, the coverage models are separated by 24.9 points of LOS in the multi-room world (59.1% vs. 84.0%), attributing the deficiency to the disk coverage model itself rather than to the acceptance rule attached to it.

## Comment 6 — References mostly pre-2020

> *The references should be updated as the majority of the references were published before 2020. Please update the references to better reflect the current state of the art.*

**Response.** We added six recent works and cite them where they bear on the argument: Zeng et al. 2020 (view-planning survey); Schmid et al. 2020 (sampling-based online informative path planning); Zhou et al. 2021 (FUEL, incremental frontier exploration); Dehbi et al. 2021 and Knechtel et al. 2025 (ISPRS J. Photogramm. Remote Sens.; optimal TLS scan planning with network-connectivity constraints, and joint standpoint + routing optimization for stop-and-go scanning with network redundancy). Together with the already-cited 2020+ works (Placed et al. 2023; Lluvia et al. 2021; Park et al. 2024; Aryan et al. 2021; Halder & Afsari 2023; Quin et al. 2021; He et al. 2025; Macenski et al. 2020), 16 of the 39 reference entries are now dated 2020 or later; of these, 13 are peer-reviewed archival publications (2020–2025), the remainder being the scanner datasheet, an open-source software reference, and our own conference paper accepted at ICCAS 2026. The two ISPRS papers anchor the related-work discussion of current TLS scan-planning practice.
