# electronics-4525706 — Reviewer Reports (received 2026-09-01, verbatim)

## Checkbox summaries

| Question | R1 | R2 | R3 |
|---|---|---|---|
| Introduction sufficient background/references | Can be improved | Can be improved | Yes |
| Research design appropriate | Can be improved | Must be improved | Yes |
| Methods adequately described | **Must be improved** | **Must be improved** | Can be improved |
| Results clearly presented | Can be improved | Can be improved | Can be improved |
| Conclusions supported by results | Can be improved | Can be improved | Can be improved |
| Figures/tables clear and well-presented | Can be improved | Can be improved | Can be improved |

## Reviewer 1

Overall: This manuscript proposes TLS-SMF, integrating terrestrial laser scanning, geometric decomposition, multi-view VLM verification, semantic knowledge graphs, and incremental object updating for robot-oriented environment modeling. The framework is relatively complete and the multi-stage verification/update design is a strength. However, several methodological details remain insufficiently defined, and the distinction between method development and independent validation needs to be strengthened.

1. The TLS preprocessing part needs a few more details. For example, the authors should explain how the RANSAC process is stopped, how floor/wall/ceiling planes are identified, and how the point cloud is normalized.
2. The structure-remnant filter is described a bit too qualitatively. Terms such as "thin vertical planar band" and "dense horizontal layer" should be supported by clear numerical thresholds so that others can reproduce the method.
3. The VLM voting and fusion process should be explained more clearly. In particular, please give the exact confidence-weighted fusion rule and explain why 0.6 was chosen as the final verification threshold.
4. It would be useful to test whether the provisional Uni3D label affects the VLM decision. Since this label is given to the VLM as input context, an ablation without this label would help show whether the VLM is truly correcting Stage-A errors.
5. The object matching strategy across different scans needs clarification. If several old objects satisfy the matching conditions at the same time, how is the final match selected? The choice of the 0.5 m, 30%, and 6 m thresholds should also be explained.
6. The DBSCAN sensitivity analysis may be somewhat circular. The selected operating-point segmentation is used as the reference when evaluating other parameter settings. An independent object-level reference would make this comparison more convincing.
7. The ground-truth labeling process may introduce some bias. Since the owner saw the proposed model labels before making corrections, the authors should discuss this possible anchoring effect and, if possible, provide independent or blinded annotations.
8. Some important corrective steps are introduced mainly in the Experiments section. Procedures such as re-splitting, the structure-condition ensemble, and owner-driven correction should be described more clearly in the Methods section, including when they are triggered and which parameters are used.

## Reviewer 2

This paper presents TLS-SMF, a framework that converts registered terrestrial laser scans into a confidence-aware, incrementally maintained semantic knowledge graph. The system is practically relevant and the failure analysis is valuable, but several central claims require stronger validation.

1. The geometric decomposition is explicitly non-novel, while multi-view VLM classification and TOSM build on [30] and [13–15]. The paper should distinguish new algorithms from system integration and quantify the additions over these prior works.
2. The exact confidence-weighted fusion rule is not provided, and the fixed 0.6 threshold is not analyzed. The drop of unanimous precision to 57.1% in Table 9 shows that the verification tiers are scene-dependent.
3. Table 4 does not adequately validate the segmentation backbone. Its proxy metrics do not measure instance quality, and semantic PTv3 outputs are not directly comparable with DBSCAN instances. The parameter sweep is also evaluated against the operating-point decomposition itself.
4. The matching rules use fixed spatial, extent, type, and distance thresholds, but conflict resolution is unspecified. The real three-epoch study does not report complete matching accuracy, identity-switch, false-new, and false-absent statistics.
5. Zero-cost re-verification is supported by identical-input or controlled-repeat tests, while the 3.1% result comes from a controlled combined-edit scenario rather than the real T2-to-T3 transition.
6. Table 5 is statistically underpowered, gated querying still has a 40.6% hallucination rate on the robot hall, and Table 7 reaches 85% recall with only 51% precision and approximately 31% type accuracy.
7. The evaluation uses several different cluster sets and denominators. A sample-flow table should connect the 38-object, 50-scored-cluster, 41-cluster, 35-cluster, and 46-object evaluation sets.
8. Several corrective modules were developed from failures observed in the same environment used for retrospective evaluation. Their complete definitions should be moved to the methodology section, and the final frozen pipeline should be evaluated independently.

## Reviewer 3

1. What is the fundamental question addressed by the research?
The research question of the article is: How can geometric data obtained from registered terrestrial laser scans be transformed into a queryable semantic knowledge graph—one that prevents unverified semantic labels from influencing robot decisions and allows for updates while preserving object identities as the environment changes?

2. Is the topic original and timely; does it address a genuine research gap?
The topic of the article is timely. It holds critical importance in the fields of robotics, 3D semantic mapping, VLM verification, knowledge graphs, and long-term environment modeling. The article aims to present a novel system by integrating components known individually—namely TLS processing, DBSCAN, open-set classification, VLM voting, knowledge graphs, and change matching. Essentially, it does not propose a new fundamental classification or segmentation algorithm; rather, it introduces a confidence-controlled, rectifiable, and identity-preserving semantic maintenance architecture. Future studies should be conducted on a wider variety of buildings within industrial environments.

3. Contribution of the study compared to previous work
The primary contribution is the consistent integration of known components around trust status, data provenance, revision history, and persistent object identity. The claims made in the paper have generally been substantiated. However, the claim regarding a new dataset/benchmark was insufficient. Furthermore, the raw scans were not shared.
It was not demonstrated that the deterministic geometric backbone remains deterministic across different operating systems, library versions, or hardware configurations.
The trust-based query gate was constructed using a small dataset, which was deemed insufficient.

4. Necessary methodological improvements
The methodology section of the article is presented in detail. However, the evaluations were based on errors observed in a specific environment, and testing was conducted within that same building. There is insufficient evidence to demonstrate whether the study would achieve the same level of success in different buildings. A test set comprising multiple buildings and independent facilities should be included. The article should also present the object class distribution for the scenes and the results for each class.
Object records, the matching function, confidence states, and upsert decisions have not been consolidated under a single formal definition. It is not clearly explained which dataset—training, development, or test—was used to determine the thresholds.
Ground truth labels should be generated in a blind manner by independent evaluators who do not see the model's suggestions.
Controlled experiments regarding registration noise, voxel resolution, and color variation should be included.
The metrics for Top-1 accuracy, per-tier precision, hallucination rate, and updates are considered quite reasonable. However, Recall/F1 per verification tier is missing. Metrics such as task success rate, path length, latency, and safe stopping are also missing.
End-to-end task validation on a real robot is not provided.

5. Are the results consistent with the presented evidence?
The results are generally consistent with the presented evidence. The experiments should be expanded. Reliability in general industrial environments is not supported. If manipulation is possible, it should be tested experimentally.

6. Appropriateness and adequacy of references
The references are generally appropriate for the topic and cover the fields of semantic mapping, scene graphs, open-set 3D recognition, VLMs, knowledge graphs, and incremental mapping. Recent studies have been reviewed.

7. Tables and figures
The tables and figures are generally satisfactory. The following improvements should be made:
- Table 2: It should be clearer at the cell level which publication or experimental evidence the indicators are based on.
- Table 6: The number of questions, question categories, and the impact of abstentions on accuracy calculations should be presented more prominently within the table.
- Figure 17: Error bars are missing from the bottom panel.
- Figure 18: The nodes where the proposed framework alters decisions should be visually highlighted.
- Figure 19: The text should clarify that this does not replace real-world robot validation.
