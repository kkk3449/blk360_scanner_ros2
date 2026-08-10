# Cover Letter — Revised Manuscript (sensors-4464625)

Dear Editor,

Thank you for the opportunity to revise our manuscript, now entitled **"Occlusion-Aware Visibility Coverage for Robotic Stop-and-Scan 3D LiDAR Mapping"** (previously *"Occlusion-Aware Visibility Coverage for Stop-and-Scan 3D Mapping with a Terrestrial Laser Scanner"*; the new title was suggested by Reviewer 1 and we have adopted it). We are grateful to the three reviewers for their careful and constructive comments, all of which we have addressed. Point-by-point responses to each reviewer are provided in the accompanying files (`response_reviewer1/2/3`, also combined in `response_to_reviewers.pdf`), together with a manuscript version with all changes highlighted (additions in blue, deletions in red strikethrough).

The principal changes are:

1. **A new experiment**, requested independently by Reviewers 1 and 2: a disk baseline governed by the *identical* marginal-gain accept/skip rule as the proposed policy, replayed on the same fixed candidate paths as the controlled ablation. The comparison is now four-way (uniform no-skip / disk spacing / disk marginal-gain / ray-cast visibility; Table 2, Figure 8). Under the identical rule, the disk coverage model reaches only 59.1 ± 1.3% LOS in the multi-room world versus 84.0 ± 9.2% for the visibility model, isolating the coverage model itself—rather than the acceptance rule—as the decisive factor. This strengthens the paper's central claim.

2. **Explicit scoping of the contribution as 2D scan-station planning for subsequent 3D mapping** (Reviewer 1): a new Section 3.5 discusses the 2D representation and vertical occlusion, and the scoping is now consistent across the title, abstract, contributions, discussion, and conclusions. The abstract's concluding claim has been softened accordingly.

3. **An explicit research question with three subquestions and a structure roadmap** now close the Introduction (Reviewer 1).

4. **Clarified formalism** (Reviewers 1 and 2): grid domain, cell centers, resolution, and threshold ordering are defined; the candidate pose is now denoted *p*, distinct from grid cells *c*; the recomputation (not freezing) of prior visibility regions on the live map is stated; Equation (6) is introduced as a normalized marginal gain defined only for nonempty regions; the square-cell assumption of Equation (7) is stated; and the ideal-geometry status of Equation (3) versus its ray discretization is clarified, as is the rationale for the occupancy thresholds 25/65.

5. **Figures revised**: the architecture diagram was redrawn with larger labels and higher color contrast; the parameter sweep now shows per-path variability with an explicit legend and a stated knee-selection criterion; the platform figure was replaced by an annotated photograph plus a dimensioned top-view schematic; the test room's dimensions and characteristics were moved into the main text; captions were shortened throughout; figures are numbered in citation order; and registration errors are reported in millimeters.

6. **References updated** (Reviewer 2): six recent works were added (Zeng et al. 2020; Schmid et al. 2020; Zhou et al. 2021; Dehbi et al. 2021; Knechtel et al. 2025, among others); 16 of the 39 reference entries are now dated 2020 or later.

7. **Conference-paper status**: since the original submission, the preliminary conference paper that this article extends has been **accepted for presentation at ICCAS 2026** (Regular Paper, decision of 31 July 2026). The overlap between the two manuscripts is now disclosed item by item in the Introduction—no figure, table, or quantitative result is shared—and the reference, the Introduction, and the first-page extended-version footnote have been updated from "under review" to "accepted". As requested by Reviewers 1 and 2, the accepted conference manuscript is provided for the editors and reviewers in the non-published material.

The non-published material archive contains the accepted ICCAS conference paper, the manuscript with changes highlighted, and the point-by-point responses; none of these files are intended for publication.

All authors have approved the revised manuscript and this submission. The manuscript is not under consideration by any other journal.

Thank you for your consideration.

Sincerely, on behalf of all authors,

Tae-Yong Kuc (Corresponding Author)
Department of Electrical and Computer Engineering,
Sungkyunkwan University, Suwon, Republic of Korea
tykuc@skku.edu
