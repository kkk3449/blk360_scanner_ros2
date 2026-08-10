# Response to Reviewer 3 — sensors-4464625 (Major Revision)

**Manuscript:** Occlusion-Aware Visibility Coverage for Robotic Stop-and-Scan 3D LiDAR Mapping

We thank the reviewer for the careful annotation of the manuscript PDF. We identified twelve annotated comments; each is quoted below with the passage it marks, followed by our response. All changes are visible in the marked-up manuscript. (Page numbers refer to the reviewed submission.)

---

## Comment 1 (p. 1) — "this is difficult to decipher"

*(on the sentence "…a dense, accurate point cloud is becoming an infrastructural asset that downstream robots localize against, plan collision-free motion within, and reason over.")*

**Response.** The sentence has been split and simplified: "…is growing. A dense and accurate point cloud is becoming an infrastructural asset: downstream robots localize against it, plan collision-free motion within it, and reason over it."

## Comment 2 (p. 1) — "the question of why isn't answered here — time, safety, less of an interruption to processes?"

*(on "…rather than through labor-intensive manual scanning.")*

**Response.** Exactly these motivations are now stated in Section 1: "The motivation is threefold: a manual survey occupies a trained operator for hours of tripod placements; a robotized acquisition can run outside operating hours, interrupting neither production processes nor people on the floor; and an autonomous procedure makes periodic re-scans repeatable at low marginal cost."

## Comment 3 (p. 2) — "this is an issue with that particular piece of hardware, but not all 3D scanners; this should be clarified because it infers that the problem is universal."

*(on "the dense scans are not registered in real time but aligned offline. Conventional mobile mapping…")*

**Response.** Agreed. Section 1 now scopes the two defining properties explicitly to the sensor class rather than to 3D scanning at large: "We emphasize that these two properties characterize the survey-grade stop-and-scan class, not 3D laser scanning in general: continuous mobile-mapping scanners acquire under motion and register incrementally online through simultaneous localization and mapping (SLAM), trading survey-grade accuracy for acquisition speed." The related claim in Section 2.3 is scoped the same way ("Survey-grade terrestrial scanning, as practiced with the tripod-class instruments considered here, departs from the continuous-acquisition assumption… (Scanners that relax these constraints exist, but at accuracies below the survey grade targeted here.)"); see also Comment 8.

## Comment 4 (p. 2) — "What do you mean here — SLAM? More precision in language is needed"

*(on "Conventional mobile mapping, which assumes continuous acquisition under motion and incremental online registration…")*

**Response.** Yes — SLAM-based continuous mapping was meant, and it is now named: "…register incrementally online through simultaneous localization and mapping (SLAM)… SLAM-based continuous mobile mapping therefore does not transfer directly to the stop-and-scan workflow addressed here." (SLAM is now defined at this first use.)

## Comment 5 (p. 2) — "what is the timeline for this?"

*(on the contribution bullet "per-run offline registration of the autonomously acquired BLK360 scans…")*

**Response.** Section 6.7 now states when and how long the registration step takes: "Registration is a post-mission batch step; importing and aligning one run's network took on the order of minutes on a desktop workstation, small compared with the acquisition time of the scans themselves."

## Comment 6 (p. 3) — "What are the restrictions of the path (dimensions)?"

*(on "…the act of 'observing' is effectively free along the path.")*

**Response.** The physical restrictions on the robot's paths are now stated in Section 5.1: "All navigation is subject to the platform's physical constraints: Nav2 plans on an inflated costmap, so every path and every scan candidate keeps the robot footprint (Figure 2b) clear of obstacles by the safety margin of Table 1 (obstacle dilation radius)." The platform's dimensions themselves are now given in the new annotated platform figure (see Comment 10): deck 71 × 49 cm at 31 cm height, wheelbase 28 cm, track 44 cm.

## Comment 7 (p. 3) — "but the origin point is lower than a typical field of view; what are the challenges/drawbacks for this?"

**Response.** A fair point that we now address in two places. Section 5.1 quantifies the vantage: "The deck places the scanner's optical center roughly 0.5 m above the floor — lower than a typical 1.5–1.8 m tripod setup." The Discussion (Section 7) analyzes the consequence: "the deck-mounted scanner observes from roughly 0.5 m above the floor, lower than a typical tripod setup, so low furniture and equipment cast longer horizontal shadows than in a manual survey of the same room. This makes occlusion-aware placement more important on a robotic platform, not less; but it also means the acquired clouds have a lower vantage than a conventional tripod survey would provide."

## Comment 8 (p. 3) — "again — this assumption can be easily challenged"

*(on "each measurement requires a stationary dwell of minutes and registration is performed offline")*

**Response.** As in Comment 3, the claim is now scoped to the instrument class rather than stated universally: "Survey-grade terrestrial scanning, as practiced with the tripod-class instruments considered here, departs from the continuous-acquisition assumption… (Scanners that relax these constraints exist, but at accuracies below the survey grade targeted here.)"

## Comment 9 (p. 9, Figure 1) — "colors too similar"

**Response.** The architecture diagram has been redrawn (also at Reviewer 1's request): the visibility-coverage block now uses a clearly darker fill and a heavy dark-blue border that separates it from the pale exploration boxes, the sequencer band uses a distinct warm tone, the survey product is green, and the redundant color legend was replaced by direct container labels. Labels were also enlarged roughly twofold.

## Comment 10 (p. 10, Figure 2) — "This angle is difficult to understand the full scale and layout of the unit — it would useful to see an annotated set of shop drawings or a more axonometric view, with dimensions and/or scalar elements"

**Response.** The platform figure has been replaced by a two-panel figure: (a) the photograph annotated with callouts for every component (Leica BLK360 G1, rigid scanner mount, YDLiDAR TG30 2D LiDAR, RGB-D camera, deck, chassis with battery/controllers/BLDC skid-steer drive); and (b) a dimensioned top-view schematic drawn from the platform's hardware specification: deck 71 × 49 cm at 31 cm height, wheelbase 28 cm, track 44 cm, wheels Ø21 × 6 cm, 2D LiDAR 7 cm behind the front deck edge, with the scanner positions marked and the scanner optical-center height (≈ 0.5 m) noted.

## Comment 11 (p. 11, Figure 3) — "dimensions? characteristics?"

**Response.** The test room's characteristics are now given in the main text (Section 5.2) rather than the caption, including its extent: "Its mapped free space spans approximately 16 × 8.5 m, about 105 m² of free floor area on the reference map," together with the qualitative characteristics (open exhibition/staging space, flat reflective floor, display wall of monitors and product panels, freestanding equipment cases and furniture creating self-occlusion, floor markings for the survey reference layout).

## Comment 12 (p. 13, Figure 5) — "explain colors"

**Response.** The caption now defines the color convention, once for all placement figures: "Each accepted scan pose (marker) and its visibility polygon share one color, indexed by scan order (#1…#4); the grayscale background is the occupancy map (light gray free, black occupied, darker gray unknown) and the blue line is the robot path. The same color convention is used in all placement figures."
