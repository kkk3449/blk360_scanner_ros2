# Code-verified pipeline parameters (for Methods rewrite, R1.1/1.2/1.3/1.5, R2.2/2.4, R3.4c)

Source: blk360_seg (= tls-smf) repo, extracted 2026-09-01 with file:line refs. Use THESE values in the manuscript — several config values are decoys (see Discrepancies).

## 1. Stage-A backbone (in-pipeline)

**RANSAC structure removal** (`blk360seg/preprocess.py:56-78`, called with all defaults from `extract_objects.py:66`):
- Custom seeded RANSAC (seed 0, deterministic): 300 random triples per plane, argmax inliers, one SVD refit on inliers, re-select.
- distance threshold **0.05 m** (NOT the 0.04 in configs/default.yaml:26 — that belongs to GeometricBaseline, never run).
- Stopping (first to fire): (i) max **10** planes; (ii) remaining points < **2000**; (iii) inliers < **4% of currently-remaining** points; (iv) plane orientation "object-like": 0.2 < |n_z| < 0.85 → stop.
- Plane classification: horizontal if |n_z| ≥ **0.85**, vertical if |n_z| ≤ **0.2**; binary structure/non-structure only (no floor/wall/ceiling distinction in-pipeline).

**Normalization**: `preprocess.normalize` (XY mean-center + z-min shift) is **NEVER CALLED in Stage A** — config `normalize: true` is unread. No global yaw alignment. Floor-z estimators used instead, per context: densest 5cm z-bin lower half (structure_filter.py:21-29); 25th-pct of cluster bottoms (structure_noise_filter.py:99); 1st z-percentile (precut/--floor-pct 1.0).
→ Manuscript must NOT claim cloud normalization; describe per-object yaw via PCA (semantic_object.py:49-66) and cross-epoch FPFH+ICP (register_epoch_scan.py: VOXEL 0.08, RANSAC 4M iters conf 0.9999 seed 0, ICP thresh 0.096).

**Offline data-prep variant** (data_prep/preprocess.py, produced stage2_no_wall.e57 for T1): Open3D segment_plane(0.03m, 1000 iters), max 20 planes, min 150k pts absolute; ceiling |n_z|>0.9 & centroid>z_mid, removed by 0.03m thickness slab; wall |n_z|<0.2 AND centroid within 12% margin of room edge (interior vertical planes kept); half-space slice removal; floor-remnant re-detect guard 10cm.

**Clustering** (configs/default.yaml, read by extract_objects.py:59-101):
- voxel **0.03 m**; DBSCAN eps **0.30 m**, min_points **100**; footprint split trigger **2.5 m** (max of x/y ptp); re-split eps **0.20 m** min_points **100**; split accepted only if ≥2 subs each ≥100 pts, else keep intact.
- Sensitivity sweep grids: eps (0.25,0.30,0.35) × minpts (50,100,150) × foot (1.5,2.0,2.5,3.0); scoring IoU≥0.5 recovered, ≥20% overlap split/merged (sensitivity_sweep.py:35-37,120-133).

## 2. Structure filters

**Ceiling-soffit filter** (structure_filter.py:40-92, all defaults): flags only if ALL pass —
1. z-extent ≤ **0.6 m** AND bottom ≥ **1.5 m** above floor;
2. thin vertical planar band: plane |n_z| ≤ **0.25** AND RMS off-plane thickness ≤ **0.03 m** (SVD);
3. above-band points: footprint padded **±0.3 m**, band zmax+0.02 … zmax+0.45, ≥ **40 pts**;
4. dense horizontal layer: 0.035 m z-bins, modal-bin slab (~7 cm) fraction ≥ **0.6**;
5. topmost-layer test: points in slab_top+0.1 … +1.5 must be ≤ **20%** of slab count (else = sill/wall, keep).

**Hybrid grid-map filter** (structure_noise_filter.py, defaults = recorded runs): wall-eps **0.15 m**, wall-hug-frac **0.6**, sheet-thin **0.22 m** (minor ext = 4√λ_min), ceil-bottom **2.0 m**, outside-frac **0.5**, full-height z-span **2.5 m**; minor_horizontal |evec_z|<0.4; local sheet thickness = median over 1.0 m PCA-axis bins (≥30 pts each); full-height band additionally needs local_thin < **0.3** (hard-coded) AND length along wall > **2.0 m** (the wall-panel exception, added commit 62da08b). Room polygon = binary_fill_holes(free∪occ) so wall-flush objects aren't "outside". Decision = strict if/elif cascade: outside → ceiling → wall-remnant → full-height band.

**Diffuseness gate + erosion resplit**: fill < **0.30** & area > **5.0 m²** (10 cm grid); erosion core cell = pts ≥ **8** & 8-neighbors ≥ **3** (v2/ensemble; the 12/4 pair is the v1 pilot ONLY); evidence gate v2: ≥250 pts, ≥5 m² (desk exception 12 m²), height ≥0.15.
**Wall-compound gate** (v2:368-373): hug=(d<0.12).mean() ≥ **0.25** AND off-wall pts ≥ **250** AND major span ≥ **2.0 m** AND not diffuse. Late wall sheet drop: median wall dist < 0.18 AND minor < 0.15. Compound split: 15cm cells span>1.3m grounded(<floor+0.35) = poles; support plane z-hist peak 0.5–0.95m; under-slab re-attach ≥ overlap 0.6 & top ≥ zd−0.30. Height levels: lidar z 0.35, ceil−1.8; high if zlo_rel>1.8, low if zlo_rel<0.25 & zhi_rel>0.175, else mid.
**Point precut**: ceiling band floor+**2.4 m** ↑ (tallest real object ~2.2 m), room mask dilation **0.35 m**.

## 3. Stage-B fusion (vlm_late_fusion.py, duplicated in analyze_escalation.py)

- **4 base views** (azim 0/90/180/270, 512px elev 25 zoom 1.9 pt-size 8); each view = one independent forced-tool call returning {type, confidence∈[0,1], reason}.
- **Fusion rule** (verbatim semantics): per-label score = **sum of that label's per-view confidences** (labels lowercased/stripped; no synonym mapping at fusion time); winner = argmax; **share = score[top]/Σ all scores**; unanimous ⟺ exactly one distinct label. Ties break on first-seen order.
- **Escalation**: triggered by ANY disagreement (not confidence). Adds **8 calls**: 4 diagonal azimuths (45/135/225/315, normal zoom) + 4 cardinal zoom-ins (_z). Re-fuse over **all 12 votes** pooled.
- **0.6 = --resolve-share default**: gates STATUS only (label always emitted): share ≥ 0.6 → verified_escalated (with escalation) / verified_majority (without); else unverified. Unanimous → verified, share not consulted. All recorded runs used 0.6.
- Second 0.6: build_det4 tie-break acceptance (answer ∈ disputed pair AND conf ≥ 0.6 → verified_recovery).
- Implicit-properties call: +1 per object on base 4 images. Synonym handling: evaluation-time equivalence sets (analyze_escalation.py:34-48, scoring only) vs pipeline-time canonical map + substring agreement (build_det4.py:44-95, agree ⇒ verified_recovery, conf=max(open,domain), clutter excluded).

## 4. KG matching/upsert (blk360seg/kg.py)

- **Identity**: anchor id = SHA1(mapId + (x,y,z,l,w,h) quantized **0.25 m**)[:10], minted once at birth; matching geometric, never id-based.
- **_matches** (pass 1): 3-D centroid dist ≤ **0.5 m** AND per-axis extents (l,w,h) within **max(30% of larger, 0.15 m)**. NO type constraint.
- **_same_object_moved** (pass 2): exact same-type (case-insensitive), type ∉ {clutter, unknown, misc, object}, same extent tolerance, **2-D** horizontal dist ≤ **6.0 m**.
- **Conflict resolution**: greedy, record-order; many-old-for-one-new → nearest **XY** centroid; many-new-for-one-old → first record claims (claimed set), later ones fall to moved pass then insert. NOT Hungarian; order-dependent, determinism inherited from Stage-A record ordering. Same greedy structure in pass 2.
- **Statuses**: unchanged (no field diff among type/confidence/status/implicit/color/pose/dimensions) / updated (≥1 diff; also anchor-hash revival of absent node) / moved (pass-2 hit, history "event":"moved") / inserted (new anchor) / absent (unclaimed live node; never deleted). revision+=1 per upsert; edges fully recomputed each revision.
- **Tiers** (_gate accepts 5 verified): verified (unanimous) / verified_escalated (esc share≥0.6) / verified_majority (no-esc share≥0.6) / verified_recovery (consensus/tie-break/agreement-gated recovery) / verified_owner (owner declaration, conf=1.0, beats VLM) + unverified (default) + refuted_structure (owner-refuted, presence=absent).
- Selective re-verification: **MD5 of cluster .ply** match ⇒ carry 12 props verbatim; else re-verify.

## Discrepancies found (fix or avoid claiming in the manuscript)

1. RANSAC dist thresh is **0.05 m** (0.04 in yaml = dead code path).
2. **No cloud normalization runs in Stage A** — don't claim floor-z alignment/XY centering; no global yaw alignment exists.
3. Erosion core 12/4 = v1 pilot only; published numbers use **8/3**.
4. moved-match distance is 2-D; primary match is 3-D.
5. Conflict resolution greedy/order-dependent — state honestly (determinism from record ordering).
6. Fifth tier verified_majority exists (+refuted_structure); kg.py docstring stale.
7. postprocess.py function defaults are decoys; YAML values (2.5/0.20/100) are what ran.
