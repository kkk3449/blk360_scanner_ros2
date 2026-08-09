# Response to Reviewers — sensors-4464625 (Major Revision)

**Manuscript:** Occlusion-Aware Visibility Coverage for Robotic Stop-and-Scan 3D LiDAR Mapping
(previously: *Occlusion-Aware Visibility Coverage for Stop-and-Scan 3D Mapping with a Terrestrial Laser Scanner*)

We thank both reviewers for their careful and constructive reviews. We have revised the manuscript throughout; the most substantial changes are:

1. **A new experiment** directly requested by both reviewers (R1 Comment 7, R2 Comment 5): a **disk marginal-gain baseline** that applies the *identical* accept/skip rule as the proposed policy to the isotropic-disk coverage model, replayed on the same fixed candidate paths as the ablation. The four-way comparison (uniform / disk-spacing / disk-gain / visibility) now appears in Table 2, Figure 8, and Sections 4.2, 6.2, and 7. The result strengthens the paper's claim: under the identical rule, the disk model achieves only 59.1 ± 1.3% LOS in the multi-room world versus 84.0 ± 9.2% for the visibility model — a 24.9-point gap attributable to the coverage model alone.
2. **Explicit 2D scoping** of the contribution (new Section 3.5, plus title, abstract, contributions, discussion, and conclusions), an explicit **research question with subquestions and a roadmap** (Section 1), the reviewer-suggested **title**, and a full pass on terminology, units (mm), abbreviations, figure order, caption length, and English.
3. **Updated figures**: the parameter sweep (Fig. 11) now shows per-path variability with an explicit solid/dashed legend; the architecture diagram (Fig. 2) was redrawn with substantially larger labels; the summary chart (Fig. 8) shows the four-way comparison.
4. **Six recent references (2020–2025)** added, including two directly relevant ISPRS scan-planning papers (Dehbi et al. 2021; Knechtel et al. 2025).

Line references below are to the revised (clean) manuscript. All changes are also visible in the marked-up version.

---

## Reviewer 1

### Major Comment 1 — Positioning: mobile robotics vs. surveying/geomatics terminology; title

> *Please clarify whether the contribution is intended specifically for robotic platforms carrying survey-grade terrestrial laser scanners or for robotic stop-and-scan LiDAR mapping more generally. ... A possible revised title can be "Occlusion-Aware Visibility Coverage for Robotic Stop-and-Scan 3D LiDAR Mapping".*

**Response.** We agree, and we have adopted the reviewer's suggested title verbatim. The contribution is intended for robotic stop-and-scan LiDAR mapping in general; the survey-grade TLS (BLK360 G1) is the canonical instance of that sensor class and our experimental instrument. The revised Introduction now defines the regime explicitly:

> "We refer to this regime as *robotic stop-and-scan LiDAR mapping*: a mobile platform carries a light detection and ranging (LiDAR) sensor that must remain stationary during each acquisition, and the dense scans are aligned offline rather than registered incrementally in real time. A terrestrial laser scanner (TLS) is the canonical instance of such a sensor; the Leica BLK360 G1 used in our experiments ... The method developed in this paper is formulated for the general stop-and-scan LiDAR regime; the BLK360 G1 enters only as the experimental instrument (Section 5)." (Section 1)

Terminology has been made consistent throughout: the method sections (3–4) now speak of the generic stop-and-scan sensor with range bound R and stationary acquisition cost; instrument-specific values appear only in the experimental sections (see also our response to Comment 4).

### Major Comment 2 — Explicit research questions and a roadmap

**Response.** The Introduction now states one principal research question and three subquestions, and closes with a roadmap:

> "**RQ.** *In online robotic exploration of an unknown indoor environment with a stop-and-scan LiDAR sensor, does an occlusion-aware visibility coverage model produce better scan placement — higher genuinely observed coverage at a comparable number of stationary scans — than the conventional proximity-based disk model?*"

with subquestions (RQ1) how per-pose coverage should be defined on the live map so it never claims occluded or unmapped space (Section 3); (RQ2) how much of the placement difference is attributable to the coverage model alone, once exploration stochasticity *and the acceptance rule* are controlled for (Sections 4, 6); and (RQ3) whether the effect carries over to a physical platform and to the registered survey product (Sections 6.6, 6.7). A structure roadmap paragraph now ends Section 1.

### Major Comment 3 — 2D visibility model vs. 3D point-cloud product

> *The authors should either extend the method toward a genuinely 3D visibility representation, or explicitly define the contribution as 2D scan-station planning for subsequent 3D mapping, discuss this limitation, and identify full 3D visibility reasoning as future work.*

**Response.** We have taken the second option, and we now state it explicitly and consistently:

- **New Section 3.5 "Scope: 2D Station Planning for 3D Mapping"** states that the model reasons on a horizontal slice at sensor height; that LOS coverage certifies floor-space sight lines, not 3D surface coverage; that vertical occlusions (shelves, overhangs, elevated machinery, height-stacked surfaces) are not represented; why the 2D scope is matched to the online setting (the occupancy grid is what SLAM provides in real time; the 2D polygon is cheap enough to evaluate at every candidate; a full-dome scanner observes much of the vertical structure from a pose with clear 2D LOS); and that a genuinely 3D visibility representation (voxel-, octree-, or surface-based) is future work.
- **Abstract**: "...a ray-cast visibility region computed on the robot's live two-dimensional (2D) occupancy map ... The planner thus performs 2D scan-station placement for subsequent 3D mapping."
- **Contributions**: the first contribution is now "defined on the live 2D occupancy grid".
- **Discussion**: the first limitation is now the 2D representation and vertical occlusion, with 3D visibility identified as the principal future-work direction.
- **Conclusions**: "...the model performs 2D scan-station planning on the live occupancy map for subsequent 3D mapping"; future work includes the 3D extension.

### Major Comment 4 — Method formulated independently of the BLK360

**Response.** Done. The method sections no longer reference the instrument: Section 4's cost motivation now reads "minutes per acquisition for the survey-grade scanner used in our experiments (Section 5)" instead of quoting BLK360 scan times; the coverage-model figure caption (Fig. 1) and the concept figure title now refer to a generic scan pose; the summary and sweep figures label "stationary scans" rather than "BLK360 scans". BLK360 duration, accuracy, and workflow details appear only in Section 5 (experimental setup) and the hardware sections, as experimental settings.

### Major Comment 5 — Broad claims, references, sensor grouping, abstract strength

**Response.** Three changes:

1. *Disk-model convention claim*: now supported with citations where first made — "a sensor-footprint approximation widely used to drive online decisions in frontier exploration and coverage practice [Yamauchi 1997; Umari & Mukhopadhyay 2017; Choset 2001; Galceran & Carreras 2013; Placed et al. 2023]." The related-work section additionally cites the NBV/view-planning literature (Scott 2003; Tarabanis 1995; Zeng et al. 2020) and the visibility-based scan-planning literature (Mozaffar & Varshosaz 2016; Prieto et al. 2017; Dehbi et al. 2021; Knechtel et al. 2025).
2. *Sensor grouping*: the family is now described as "sensors that require stationary acquisition or a dwell time at each viewpoint", and we state explicitly: "These instruments differ widely in observation geometry, resolution, and uncertainty; the line-of-sight coverage model developed here applies to the optical, non-penetrating members of this family and explicitly not to penetrating modalities such as GPR (Section 7)."
3. *Abstract claim*: softened and scoped — "The results indicate that, for the room-scale indoor environments studied, occlusion-aware visibility is a sounder basis than Euclidean proximity for stop-and-scan placement." The Conclusions carry the same scoping ("Within the room-scale indoor environments and the optical stop-and-scan sensor class studied here...").

### Major Comment 6 — Conference-paper overlap

**Response.** The Introduction now discloses the overlap explicitly and itemizes what is new:

> "The conference version contributes the stop-and-scan system integration (frontier exploration coupled to a scan sequencer) and an isotropic-disk scan trigger, demonstrated in simulation. New to this article are: the ray-cast visibility coverage model and the LOS coverage metric (Section 3); the dual-criterion marginal-gain placement rule and the coverage-completion phase (Section 4); the controlled paired ablation with its three baselines and the parameter sensitivity analysis (Section 6); and the entire hardware study ... No figure, table, or quantitative result is shared between the two manuscripts."

The sentence noting the platform's absence from the conference version has been removed, as requested. The conference manuscript (under review at ICCAS 2026) has been provided to the editorial office as a file for editors and reviewers, and we have confirmed that this extension route is consistent with MDPI's editorial policy on extended conference papers (the extension is disclosed in the manuscript and the conference paper is cited as ref. [9]).

### Major Comment 7 — Equations (1)–(7) clarifications

**Response.** All requested clarifications have been made in Sections 3–4:

- **Grid domain/resolution/ordering**: "The grid is an H×W array of square cells of side ρ, the grid resolution (ρ = 0.05 m in all experiments); each cell c is identified with its center point in the map frame, so that ‖c − c′‖ denotes the Euclidean distance between cell centers. ... thresholds ordered as 0 ≤ θ_free < θ_occ ≤ 100." (Section 3.1)
- **Cell vs. candidate pose**: the candidate pose is now denoted **p** throughout Section 4 and Algorithm 1 ("Throughout the paper, s_i denotes an accepted scan pose and p a candidate pose under evaluation, while c is reserved for grid cells", Section 3.1).
- **Frozen or recomputed**: "The visibility regions of previously accepted scans are not frozen: at every decision, C(S) is recomputed on the current occupancy map, so earlier regions expand or contract as SLAM refines the map (the final reported coverage is evaluated once more on a single reference map; Section 5)." (Section 4.1) This matches the implementation.
- **Equation (6)**: now introduced as the "*normalized marginal gain*", "defined only for a nonempty visibility region", with the degenerate-region cut stated to reject candidates *before* the ratio is evaluated, "so the denominator of the normalized gain never vanishes."
- **Equation (7)**: "where cells are squares of side ρ (the grid resolution, ρ = 0.05 m), so each cell contributes area ρ² and a is in m²."
- **Disk baseline vs. Equations (6)–(7)**: we now state explicitly that the spacing rule is *not* mathematically equivalent to the marginal-gain rule applied to B_disk ("a candidate closer than R to an existing scan can still contribute a crescent of new disk area"), and we resolve the resulting confound with a **new experiment**: a disk marginal-gain baseline applying the identical dual criterion with B_disk substituted for B, replayed on the same fixed paths (Sections 4.2, 6.2; Table 2; Figure 8). Within that pair, the only difference is the coverage model. Results: single-room 77.7 ± 3.8% LOS at 2.6 ± 0.5 scans (essentially the spacing rule's result); multi-room **59.1 ± 1.3% LOS** at 2.9 ± 0.3 scans — *worse* than the spacing rule, because the disk model's own coverage bookkeeping saturates through partitions and stops scanning while whole rooms remain unobserved. This isolates the deficiency to the coverage model itself rather than the acceptance rule.

### Major Comment 8 — Figure 11 (parameter sweep) presentation

**Response.** The figure has been regenerated and the text rewritten:

- The legend now identifies each curve explicitly: "single-room — LOS coverage (solid, left axis)", "single-room — scan count (dashed, right axis)", etc.
- All curves now carry **error bars (per-path standard deviation over the N = 5 / N = 10 replay paths)**; the caption states "mean ± s.d."
- Scan-count changes are no longer described in "percentage points": "...changes mean LOS coverage by at most about two points (single-room 86.4→84.3%; multi-room 85.2→84.0%) and mean scan count by less than one scan."
- The knee identification is now an explicit criterion: "it is the largest threshold whose mean LOS remains within about five points of the most aggressive setting (A_min = 1 m²) in both environments (85.4 vs. 90.7%; 84.0 vs. 85.7%) while already halving the scan count (3.6 vs. 7.2 scans in both environments). Beyond the knee the trade turns unfavorable: pushing to A_min = 15 m² saves only about one further scan (3.6→2.2 and 3.6→2.8) while coverage continues to erode (85.4→80.8% and 84.0→83.1%)."

### Minor Comment 9 — Geometry vs. RGB

**Response.** The Introduction now states once: "(The scanner also records panoramic RGB imagery that colorizes the point cloud; the planning and evaluation in this paper concern the geometric measurements only.)" Repeated "colorized point cloud" phrases have been removed; the word now appears exactly twice (this definitional sentence and the caption of the colorized rendering in Fig. 15, where color is genuinely shown).

### Minor Comment 10 — Long sentences / English

**Response.** The page-1 sentence has been split ("...is growing. A dense and accurate point cloud is becoming an infrastructural asset: downstream robots localize against it, plan collision-free motion within it, and reason over it."). We have additionally shortened the longest captions (Figs. 2, 3, 7, 8, 11, 13–15; Tables 2–3) and made an English pass over the manuscript.

### Minor Comment 11 — Citation after author names

**Response.** Fixed: "Mozaffar and Varshosaz [6] optimize scanner placement to reduce occlusions, and Prieto et al. [5] drive a next-best-scan policy...". The same convention is applied to the newly added named citations (Dehbi et al. [x], Knechtel et al. [y]).

### Minor Comment 12 — Abbreviations at first use

**Response.** All abbreviations are now defined at first use: 3D, TLS, LiDAR, GPR (Section 1); LOS (Section 1); NBV (Section 2.1); SLAM (Section 3.1); BIM (Section 2.3); MPPI (Section 5.1). The abbreviations list has been extended accordingly (LiDAR, GPR, BIM added).

### Minor Comment 13 — Figure citation order

**Response.** Fixed. The coverage-model figure (previously Figure 4, cited in Section 3 before Figures 1–3 appeared) has been moved to Section 3.2, where it is first cited; it is now Figure 1, and all figures are numbered in order of first citation.

### Minor Comment 14 — Figure 1 (architecture) label size

**Response.** The architecture diagram has been redrawn: labels are now roughly twice as large at manuscript scale, the layout is a compact two-row pipeline, the redundant color legend was removed (the containers are labeled directly), and the caption has been shortened to two sentences with the component description kept in the Section 5.1 text.

### Minor Comment 15 — Test room described in caption only

**Response.** The test room is now introduced in the main text (Section 5.2: open exhibition/staging space, reflective floor, display wall of monitors and product panels, freestanding equipment cases and furniture creating self-occlusion, floor markings for the survey reference layout), and the figure caption has been reduced to a single sentence.

### Minor Comment 16 — Millimeter units

**Response.** All registration errors are now reported in millimeters: bundle errors 4–5 mm, per-link errors 3–6 mm, disk single-link 2 mm, quality band ≤ 15 mm. Table 3's column is now "Bundle error (mm)". Units are consistent throughout the text, tables, and abstract.

### Minor Comment 17 — Repetition

**Response.** We removed repeated explanations of colorized point clouds (now defined once), the single-link/multi-link registration contrast (explained once in Section 6.7 and only referenced elsewhere), and the disk-vs-visibility difference (defined in Sections 1/3 and thereafter referenced); shortened captions no longer duplicate the surrounding text.

*(The review text we received proceeds from Comment 17 directly to Comment 19; if a Comment 18 was intended, we would be glad to address it.)*

### Minor Comment 19 — Caption length

**Response.** Captions of Figures 2, 3, 7, 8, 11, 13, 14, 15 and Tables 2, 3 have been shortened, with interpretation and methodological detail moved into the main text (Sections 5.1, 5.2, 6.2, 6.5–6.7).

---

## Reviewer 2

### Comment 1 — Is the isotropic disk a sufficient baseline? Why no additional methods?

**Response.** We have strengthened the evaluation in two ways.

*Added comparison.* The revision adds a fourth policy: a **disk marginal-gain baseline** that applies the identical accept/skip rule as the proposed policy to the disk coverage model (see the response to Reviewer 1, Comment 7, and Sections 4.2/6.2, Table 2, Figure 8). The four compared policies are now: uniform no-skip (the fixed-interval policy of stop-and-go practice; upper reference), disk spacing (proximity rule), disk marginal-gain (matched rule), and ray-cast visibility (ours).

*Justification of the baseline set.* Section 6's preamble now justifies this set explicitly: the disk/proximity treatment is the de facto online coverage model in stop-and-go practice (Section 2), while published TLS scan-planning methods — including the strongest recent ones (Dehbi et al. 2021; Knechtel et al. 2025, now cited) — optimize viewpoints *offline over a prior environment model* (floor plan, BIM, or coarse scan) and are therefore not applicable as baselines in the online unknown-environment regime studied here, where every decision must be made on the live SLAM map. NBV planners for continuous sensors likewise do not decide whether to pay a large fixed stationary cost. The uniform no-skip reference bounds attainable coverage along each path, so the reported gains are located within a bracketed range rather than against a single baseline.

### Comment 2 — Reference [9] (conference paper) availability and overlap

**Response.** The overlap is now disclosed item by item in the Introduction (see response to Reviewer 1, Major Comment 6: what the conference version contains, what is new here, and that no figure, table, or quantitative result is shared). The conference manuscript has been uploaded for the editors and reviewers as a supplementary file for review purposes, as requested.

### Comment 3 — Why thresholds 25 and 65 in Equation (1)?

**Response.** Section 3.1 now explains: these are the long-standing free/occupied convention of the ROS occupancy-map pipeline (probability thresholds 0.25/0.65), and the classification is insensitive to their exact placement because SLAM-estimated occupancies are strongly bimodal (almost all mapped cells settle near 0 or 100), so any thresholds separating the two modes yield the same classification; the deliberately separated band maps ambiguous cells to *unknown*, which the visibility model treats conservatively (they block rays and are never counted as covered).

### Comment 4 — Does Equation (3) represent ideal geometric visibility (no K, Δθ)?

**Response.** Correct, and this is now stated explicitly after Equation (3): "the set definition above characterizes the ideal geometric visibility region and does not depend on the ray discretization; the K rays and half-cell marching are the implementation that approximates it. At the range bound the inter-ray arc spacing is R·Δθ ≈ 5.2 cm for R = 6 m, on the order of a single grid cell (ρ = 5 cm), so the discrete approximation recovers the ideal region up to single-cell effects at region boundaries."

### Comment 5 — Is the disk baseline's acceptance logic the same? Table 2's claim

**Response.** The reviewer is right, and this prompted the main new experiment of the revision. The spacing rule is *not* the marginal-gain rule applied to B_disk, and we now say so explicitly in Section 4.2. To make the comparison the reviewer asks for, we added the **disk marginal-gain baseline** (identical Equation (8) rule, B_disk substituted for B, disk-based bookkeeping of the covered set) to the controlled ablation. Table 2 now contains four rows per environment, and its caption has been corrected to: "...the disk marginal-gain row applies the identical accept/skip rule as the visibility policy, so within each environment that pair differs only in the coverage model." The former claim ("the only difference between rows is the coverage model") has been removed. Notably, the matched-rule disk baseline performs *worse* than the spacing rule in the multi-room world (59.1 ± 1.3% vs. 77.0 ± 6.4% LOS): the disk model's through-wall over-claiming saturates its own coverage bookkeeping, so the rule stops scanning while whole rooms remain unobserved (Section 6.2 explains the mechanism).

### Comment 6 — References mostly pre-2020

**Response.** We added six recent works and now cite them where they bear on the argument: Zeng et al. 2020 (view-planning survey); Schmid et al. 2020 (sampling-based online informative path planning); Zhou et al. 2021 (FUEL, incremental frontier exploration); Dehbi et al. 2021 (optimal TLS scan planning with network-connectivity constraints, ISPRS JPRS); Knechtel et al. 2025 (joint standpoint + routing optimization for stop-and-go scanning with network redundancy, ISPRS JPRS); together with the already-cited 2020+ works (Placed et al. 2023; Lluvia et al. 2021; Park et al. 2024; Aryan et al. 2021; Halder & Afsari 2023; Quin et al. 2021; He et al. 2025; Macenski et al. 2020), 16 of the 39 reference entries are now dated 2020 or later; of these, 13 are peer-reviewed archival publications (2020--2025), the remainder being the scanner datasheet (2022), an open-source software reference (2026), and our own conference manuscript under review. The two ISPRS papers anchor the related-work discussion of current TLS scan-planning practice.

---

*All authors have approved this response and the revised manuscript.*
