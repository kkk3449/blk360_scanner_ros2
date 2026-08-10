# Cover Letter — MDPI Electronics submission

Dear Editors of *Electronics*,

We are pleased to submit our manuscript, **"From Terrestrial Laser Scans to
Queryable Robot Knowledge: A VLM-Verified Framework for Incremental 3D
Semantic Modeling,"** for consideration as an Article in *Electronics*.

Indoor service and inspection robots need environment knowledge that is
queryable, reliable, and maintainable as the site changes; existing
open-vocabulary 3D pipelines propagate unverified labels downstream, and
build-once scene models offer no maintenance path. The manuscript presents
TLS-SMF, a framework that converts registered terrestrial laser scans into
a confidence-aware semantic knowledge graph and keeps it current across
repeated scans. Its three core components are (i) a deterministic geometric
backbone whose re-runs are byte-identical, (ii) multi-view
vision–language-model verification with automatic escalation that retains
unresolved objects as explicitly unverified, and (iii) identity-preserving
incremental updates over mint-once node identities. The evaluation spans
segmentation repeatability, verification accuracy and per-tier precision,
structurally gated querying with hallucination traps, synthetic and real
multi-epoch updates, a cross-model verifier comparison, a cross-floor
stress scene that bounds domain transfer, and closed-loop robot mission
execution in simulation — with failure taxonomies, run-to-run variance,
and monetary cost reported throughout. We believe this
reliability-and-lifecycle perspective on robot semantic mapping fits the
robotics and intelligent-systems scope of *Electronics* well.

This manuscript substantially extends our earlier conference study on
multi-view VLM classification of TLS object clusters (Kim et al., submitted
to ICTC 2026, cited in the manuscript). The journal submission adds
automatic verification escalation with tier-precision analysis, explicit
retention of unverified content, structurally gated querying,
identity-preserving multi-epoch updates with a controlled-change re-scan
study, corrective decomposition (automatic re-split gates and a
structure-condition ensemble), owner-evidence revision with full
provenance, end-to-end robot tasking and simulation-twin integration, and
extensive cross-model and cross-condition evaluations, including a
cross-floor stress scene and a verifier-times-render-source ablation. None
of this material has been published or is under consideration elsewhere.

For transparency: the two principal evaluation sets are two operational
configurations of one industrial room (stated as such throughout); the
end-to-end replay of Section 5.7 is explicitly framed as a retrospective
evaluation on the development scene; and the ground-truth confirmation
process, performed by the facility owner with access to model-proposed
labels, is disclosed together with its anchoring limitation (Section 5.1)
and the related conflict-of-interest statement. The owner-audited ground
truth for all five evaluated cluster sets, the query benchmarks with
per-claim scoring, the fixed pipeline configuration, and the exact prompt
and scoring code are openly available in the `supplementary/` directory of
the public implementation repository referenced in the Data Availability
statement.

All authors have approved the manuscript and agree with its submission to
*Electronics*. The manuscript is not under consideration by any other
journal.

Thank you for your consideration.

Sincerely, on behalf of all authors,

Tae-Yong Kuc (Corresponding Author)
Department of Electrical and Computer Engineering,
Sungkyunkwan University, Suwon, Republic of Korea
tykuc@skku.edu
