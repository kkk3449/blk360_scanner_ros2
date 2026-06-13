# BLK360 Occlusion-Aware Visibility Stop-Scan — Context for Paper/Report Writing

> Hand-off document for Claude Chat. Summarizes the codebase and experiments so a
> writing assistant can draft a journal paper / report without re-reading the code.
> Repo: `blk360_scanner_ros2` (ROS 2 Jazzy). Workspace: `~/blk360_ros2_ws`.
> Last updated: 2026-06-13.

---

## 1. Project purpose

Automate **survey-grade 3D mapping** with a stationary terrestrial laser scanner
(Leica **BLK360 G1**) mounted on a mobile robot. The robot autonomously explores an
unknown indoor space and decides **where to stop and take a panoramic scan**
("stop-and-scan"), minimizing the number of expensive stationary scans while
guaranteeing coverage.

**Two work threads share this codebase:**
- **(A) PhD preliminary-defense thesis (done):** frontier-exploration stop-scan with
  an isotropic-disk coverage model; Chapter 6/7 written. Baseline for thread B.
- **(B) Journal paper (current):** upgrade the skip decision from an isotropic disk
  to an **occlusion-aware sensor-visibility region** (ray-cast visibility polygon),
  so coverage reflects actual line-of-sight, not just Euclidean range. This document
  is primarily for thread B.

**Journal thesis statement.** A coverage model that ignores occlusion (the disk)
over-estimates what a stationary scan actually captures, because it counts area
behind walls/obstacles as covered. Replacing it with a ray-cast visibility region
yields placements that respect line-of-sight, raising *guaranteed* coverage at a
modest scan-count cost. The gap grows as the environment becomes more partitioned.

**Motivating framing (intro):** many high-precision stationary sensors — TLS,
thermal cameras, gas sensors, ground-penetrating radar, acoustic source
localization, spectral sensors — require a stop-and-scan workflow and coverage-aware
navigation. The visibility-based stop-scan policy generalizes to any such
line-of-sight-limited stationary sensor.

---

## 2. Overall architecture

ROS 2 nodes (decoupled; coordinate via topics/services, not direct calls):

```
 Gazebo sim (testroom worlds) ──/scan, /odom, /tf──┐
                                                    ▼
 Cartographer SLAM ──/map──► frontier_exploration_ros2 ──Nav2 goals──► Nav2 stack
        │                          │ (drives the robot, owns Nav2 goals)
        │                          ▼
        │             stop_scan_sequencer (FSM)  ◄── /map (visibility model)
        │                 │  drive → STOP → SCAN → resume
        │                 │  decides skip-vs-scan per candidate
        │                 ▼
        │             mock_blk360_scanner  (scan/download timing)
        ▼
 exploration_monitor ── stall-based "exploration complete" ──► sequencer finalize
```

- **Cartographer** = 2D SLAM (occupancy `/map`).
- **frontier_exploration_ros2** = external package; picks frontier goals, drives via
  Nav2. Owns Nav2 goals during exploration.
- **stop_scan_sequencer** = our high-level FSM: watches displacement, decides when a
  candidate location is worth a stationary scan, triggers the scanner, gates on
  download so scans never overlap.
- **exploration_monitor** = declares convergence when the known map stops growing
  (no ground truth needed), then fires completion.
- **mock_blk360_scanner** = stand-in for the real BLK360 SDK in sim (CAPTURED/DONE
  timing, optional injected failures).

The same sequencer runs against the real BLK360 by swapping the mock node for the
real `blk360_scanner` driver (hardware path exists but the journal experiments are
in simulation).

---

## 3. Key modules / file structure

```
src/blk360_stop_scan/blk360_stop_scan/
  visibility.py            ~96 LOC  ★ NEW. Ray-cast visibility regions (pure numpy,
                                     ROS-free, offline-testable).
  stop_scan_sequencer.py   ~1100 LOC  Main FSM; coverage_model = disk | visibility;
                                     skip decision; coverage-completion phase; RViz
                                     markers; per-run JSON records.
  exploration_monitor.py            Stall-based completion (known-map growth).
  mock_blk360_scanner.py            Sim scanner timing/failure model.

src/blk360_bringup/
  launch/active_mapping.launch.py   Brings up SLAM+Nav2+frontier+sequencer+monitor;
                                     exposes coverage_model, tau, A_min, R, stall,
                                     coverage_completion, min_pocket_area args.
  launch/testroom_sim.launch.py     Spawns robot; world_name arg selects world.
  worlds/testroom.world             Single room (from a real BLK360 E57 scan).
  worlds/testroom_multiroom.world  ★ NEW. + two partitions → 3 occluded rooms.

scripts/
  skip_decision_ablation.py ★       Deterministic offline replay of both skip rules
                                     on a fixed map + fixed path (the clean experiment).
  render_journal_figs.py   ★        Regenerates the journal figure set fig1–8.
  render_mapping_figs.py            Thesis (Ch.6) mapping figures.
  traj_logger.py                    Logs map→base_footprint trajectory to CSV.
  run_vis*.sh / run_cc_stats.sh / run_sota_stats.sh / run_mr_*.sh
                                     One-config / N-run experiment drivers.
  metrics.py                        Overlap / coverage / IoU / wall-RMSE metrics.

outputs_thesis/journal/             fig1–8 + captions.md (figures gitignored).
docs/claude_chat_context.md         This file.
```
★ = added/central for the journal thread.

---

## 4. Core algorithms

### 4.1 Sensor-visibility region (the contribution)
For a scan pose `s` and range `R`, the covered area is the **visibility region**

```
B(s, R) = { p : |p − s| ≤ R  AND  segment s→p crosses no occupied/unknown cell }
```

computed by marching `num_rays` (default 720) rays outward on the live occupancy
grid; each ray stops at the first occupied (≥ `occupied_thresh`) or unknown cell.
This is a discrete **visibility polygon**. Contrast with the legacy
**isotropic disk** `B_disk(s,R) = {p : |p−s| ≤ R}`, which ignores walls.
Unknown cells block rays (conservative: never claim unconfirmed space).
Implementation: `visibility.py` — `visible_mask`, `union_visible_mask`,
`new_visible_ratio`. Pure numpy, ~2 ms per polygon.

### 4.2 Scan-skip decision (when to stop and scan)
At each candidate location (raised every `scan_interval_m` of straight-line travel),
let `C = ∪_i B(s_i, R)` be the union coverage of prior scans. The candidate's
**marginal gain** is `|B(c,R) \ C| / |B(c,R)|`. A scan is **taken** when

```
gain ≥ τ          (relative: enough of what it sees is new)
   OR
new_area ≥ A_min  (absolute: a meaningful new area, in m²)
```

and **skipped** only when both fail. Operating point **τ = 0.30, A_min = 5 m²**.
Why both: at BLK-scale `R` the footprint is large, so a small occlusion pocket is a
tiny *fraction* (ratio fails) yet absolutely worth scanning — `A_min` catches it;
`τ` catches large new rooms. Legacy disk rule = skip if the candidate is within `R`
(straight line) of any prior scan.

### 4.3 Coverage-completion phase (optional, "no white left")
After frontier exploration converges, optionally do not finalize while uncovered
free-space pockets remain: cluster the uncovered free cells (`free ∧ ¬C`), and for
each pocket ≥ `min_pocket_area_m2`, drive **into** the pocket with Nav2
(`navigate_to_pose`) and scan; repeat until none remain. Guards: per-pocket nav
timeout, a scan budget, and an unreachable-pocket list.

### 4.4 FSM (sequencer states)
`INIT → EXPLORING → STOPPING → SCANNING → (RECONNECT on failure) → RESUMING → …`,
plus `COVERING` for the completion phase, terminal `DONE`. Capture/download are
decoupled (robot may drive while the BLK360 downloads; the next scan is gated so two
never overlap).

### 4.5 Convergence detection
`exploration_monitor` declares "complete" when the **known map stops growing**
(< `min_progress_cells` for `stall_timeout_s` while EXPLORING) — works without ground
truth; frontier counts are only a readout.

---

## 5. Current implementation status

| Component | Status |
|---|---|
| Disk stop-scan (thesis baseline) | ✅ done, validated, in thesis Ch.6 |
| Visibility region + ray-cast (`visibility.py`) | ✅ done, offline-tested |
| `coverage_model = disk \| visibility` in sequencer | ✅ done |
| Dual-criterion skip (τ OR A_min) | ✅ done |
| Coverage-completion phase (Nav2 pocket visits) | ✅ done, validated |
| RViz visibility-polygon markers + run-JSON records | ✅ done |
| Multi-room world + `world_name` launch arg | ✅ done |
| Deterministic offline ablation harness | ✅ done |
| Journal figures fig1–8 + captions | ✅ done |
| Real BLK360 hardware run (journal) | ⬜ not done (sim only) |
| Per-point GT segmentation metrics | ⬜ thesis future work |

All in simulation (Gazebo, TurtleBot3 Waffle as the mobile base). No LaTeX on this
machine — figures rendered as PNG; thesis compiled in Overleaf.

---

## 6. Recent changes (git, newest first)

- `42e2698` Journal figure renderer (fig1–8) + captions; shared deterministic-replay
  helpers between the ablation and the parameter sweep.
- `1d22b3c` Widen multi-room doorway 1.0→1.8 m; stall 90→120 s; add
  `skip_decision_ablation.py` (the controlled experiment).
- `9e1f1e6` Multi-room world + disk-vs-visibility comparison run scripts.
- `59d36c2` **Core feature:** occlusion-aware visibility coverage stop-scan +
  coverage-completion phase; launch args; run/stats scripts.
- `cfc4e68` Trajectory logging + single-run scripts + mapping-figure renderer.

---

## 7. Experiments / how to reproduce

**Run a sim experiment** (headless): the `run_*.sh` scripts launch the sim, the
stack, log the trajectory, save the map, and write a per-run JSON to the output dir.
Key knobs passed through `active_mapping.launch.py`:
`coverage_model`, `scan_coverage_radius_m` (R), `min_new_visible_ratio` (τ),
`min_new_visible_area_m2` (A_min), `coverage_completion`, `min_pocket_area_m2`,
`stall_timeout_s`, `world_name`.

- `run_vis.sh`, `run_vis_R5/R6.sh` — single visibility runs (R sweep).
- `run_cc_stats.sh` — N runs with coverage completion.
- `run_sota_stats.sh` — N runs, visibility only.
- `run_mr_stats.sh` — N runs per model on the multi-room world.

**Controlled offline ablation (the clean experiment):**
`skip_decision_ablation.py <map.pgm> <map.yaml> <traj.csv> [R τ A_min interval]`
replays **both** skip rules on one fixed map and one fixed candidate path (the logged
trajectory, sampled every 2 m), so the *only* difference is the skip decision. This
removes exploration stochasticity (see Issue #1). The journal figures and the
parameter sweep reuse this harness.

**Regenerate all journal figures:** `python3 scripts/render_journal_figs.py`.

**Coverage metric used throughout:** *LOS coverage* = fraction of mapped free space
with guaranteed line-of-sight from at least one scan pose (= union of ray-cast
visibility regions ∩ free / free). This is the honest figure-of-merit the disk model
over-states.

### Headline results
- **Single live demo, multi-room:** disk 2 scans, 46.9 % LOS (a whole room counted
  as covered through a wall) vs visibility 3 scans, 95.1 % LOS (one scan per room).
- **Controlled paired ablation (N=5 trajectories/env, same path both rules):**
  - Single-room: disk 2.0 ± 0.0 scans / 77.0 ± 4.9 % → visibility 3.6 ± 0.9 / 85.4 ± 6.2 %.
  - Multi-room: disk 2.3 ± 0.5 / 77.0 ± 6.4 % → visibility 3.6 ± 0.7 / 84.0 ± 9.2 %.
  - Per-path LOS gain up to **+21–23 %p** on full-room-traversal paths.
- **Coverage completion (N=5):** 5.0 ± 1.2 total scans (3.2 frontier + 1.8
  completion), **98.9 ± 0.9 % LOS**, 0/5 unreachable pockets.
- **Parameter sweep:** τ is flat over [0.10, 0.50] (≈2 %p) → robust; A_min is the
  knob, with the knee at **5 m²** (coverage within ~5 %p of the aggressive A_min=1 m²
  at roughly half the scans).
- **R sweep:** R = 5/6/10 m → ~97–98 % LOS; **R = 6 m** chosen (coverage vs
  inter-scan overlap for registration).

Data lives in `~/blk360_visrun`, `~/blk360_ccstats`, `~/blk360_sotastats`,
`~/blk360_mrstats`, `~/blk360_multiroom` (per-run JSON + map PGM/YAML + trajectory
CSV).

---

## 8. Open issues / caveats

1. **Live multi-room stats are confounded by exploration stochasticity.** Which rooms
   get mapped (Nav2 + frontier, narrow doorways) dominates coverage, independent of
   the skip rule, inflating variance and shrinking the gap. → Use the **deterministic
   offline ablation** as the rigorous result; the live run is the qualitative demo.
2. **LOS coverage is computed on each run's *mapped* free space**; partial maps make
   the denominator vary. Cross-run comparisons re-evaluate every run on one common
   reference map (the most-complete one).
3. **Map-saver race:** `map_saver_cli` occasionally fails to grab the latched map;
   scripts now retry up to 3×.
4. **Never reaches 100 % LOS** — sub-`min_pocket_area` fragments at wall/obstacle
   edges remain; reported honestly as "no uncovered pocket ≥ X m²".
5. **Sim only for the journal.** Real BLK360 hardware validation is pending.
6. **`stall_timeout` is environment-sensitive:** bigger/partitioned worlds need a
   longer warmup and a longer stall window or exploration ends prematurely.

---

## 9. Ready-to-use technical paragraphs (drop into a paper/report)

**Method — coverage model.**
> We model the area captured by a single stationary scan at pose *s* with range *R*
> as the sensor's visibility region *B*(*s*,*R*) = { *p* : ‖*p*−*s*‖ ≤ *R* and the
> segment *s*→*p* is unobstructed on the occupancy map }, computed by casting
> *K* = 720 rays and truncating each at the first occupied or unmapped cell. Unlike
> the isotropic disk *B*₍disk₎(*s*,*R*) = { *p* : ‖*p*−*s*‖ ≤ *R* }, which implicitly
> assumes an obstacle-free field of view, the visibility region accounts for wall and
> clutter occlusion, so coverage reflects the scanner's true line of sight.

**Method — scan placement.**
> The robot raises a scan candidate every *d* m of travel. Given the union coverage
> *C* = ⋃ᵢ *B*(*sᵢ*,*R*) of prior scans, a candidate *c* is accepted when its
> visibility region contributes either a fraction ≥ τ of its own area or an absolute
> new area ≥ *A*₍min₎ — i.e. accept iff |*B*(*c*,*R*)∖*C*| / |*B*(*c*,*R*)| ≥ τ or
> |*B*(*c*,*R*)∖*C*| ≥ *A*₍min₎ — and skipped otherwise. The absolute term prevents
> the relative criterion from discarding small but genuinely occluded pockets, whose
> new area is a negligible fraction of the large BLK360 footprint. We use τ = 0.30 and
> *A*₍min₎ = 5 m².

**Experiment — controlled ablation.**
> To isolate the skip decision from the stochasticity of autonomous exploration, we
> replay both coverage models offline on a fixed environment map and a fixed candidate
> sequence (the logged robot trajectory sampled every 2 m), so the only variable is
> the accept/skip rule. Across five trajectories per environment, the visibility model
> raises guaranteed line-of-sight coverage from 77.0 ± 4.9 % to 85.4 ± 6.2 % in a
> single room and from 77.0 ± 6.4 % to 84.0 ± 9.2 % in a three-room layout, at a cost
> of roughly one additional scan; on paths that traverse every room the gain reaches
> +21–23 percentage points.

**Result — why the disk fails.**
> The disk model is structurally limited: with *R* comparable to the room scale, every
> candidate lies within *R* of an existing scan once two scans are placed, so it
> stops scanning even though walls occlude much of the claimed area. In the three-room
> world it leaves an entire room unscanned (46.9 % line-of-sight coverage) while
> reporting it as covered, whereas the visibility model places one scan per room
> (95.1 %).

(Adjust ± values only if you re-run; numbers above are from the committed runs.)

---

## 10. Notes for handing this to Claude Chat

- **This is a robotics / 3D-mapping research codebase** (ROS 2, Gazebo, Cartographer,
  Nav2). Frame requests as scientific writing, not code generation.
- **Two distinct deliverables share the repo:** the PhD thesis (disk baseline,
  Chapters 6–7, already drafted) and the journal paper (visibility upgrade). Keep them
  separate; this doc is for the journal paper unless stated otherwise.
- **"LOS coverage" is the key metric** — always means ray-cast line-of-sight coverage
  of *mapped free space*, not Euclidean-disk coverage. Don't let it be conflated with
  the disk model's inflated coverage.
- **Cite numbers from Section 7**; they come from committed runs. The live multi-room
  *statistics* are weak (Issue #1) — lead with the **deterministic ablation** and the
  **single qualitative demo**, not the live N-run multi-room averages.
- **Figures fig1–8** are described in `outputs_thesis/journal/captions.md`; refer to
  them by number and reuse those captions.
- **Terminology to keep consistent:** "stop-and-scan", "visibility region /
  visibility polygon *B*(*s*,*R*)", "isotropic disk", "marginal gain", "coverage
  completion", "line-of-sight (LOS) coverage", "BLK360 G1". Avoid calling *R* a
  "sensor coverage radius" — it is a range bound on the visibility region.
- **No raw code needed for writing.** If the assistant needs a formula or parameter,
  Sections 4 and the paragraphs in Section 9 are authoritative.
- **Unverified / do-not-invent:** real-hardware results, per-point segmentation
  metrics, and any number not in Section 7 do not exist yet — do not fabricate them.
