# Design decisions of the optimization model (reference draft for Section 3: System Design and for the paper)

Reference notes, not final report text. Future tense is used so that sentences can be copied into a proposal. Reference keys are BibTeX keys
(`additions2.bib` and the project `.bib`); "table row X" means a row of `parameters_reviewed.tsv`; "(no key)" means no BibTeX key exists yet.
Code locations are given so that every claim can be traced.

---

## D1. Drones will be a response and verification layer, not a detection layer

**Decision.** Detection (AD₀) will come from the DAS and from the existing CCTV cameras only. A drone will be used after an alarm: it will be sent to the event,
verify it and track the intruder (the operating sequence of the system: DAS alarm → camera or drone steering → verification → tracking).

**Why.**
1. A drone observes a small footprint at a time. Treating it as a sensor that detects over its whole operating radius (11 km, table row `Cov_drone`) would assume that one drone watches tens of cells simultaneously and continuously.
2. A drone is not always available: it flies 47 min (`T_drone`) and charges 27 min (`T_charge`, `dji_dock3_specs`), i.e. one drone is ready 63.5 % of the time and two drones at a station give 100 %.
3. The role matches the project description of the drone as a "mobile response and tracking" asset.

**Rejected alternative: drones as a detection layer** (the first version of the model, `Pd_drone` = 0.885 over the whole cell in reach). With this model the optimization removed the DAS from 27 of 139 cells
(C_total 9.82 MSAR instead of 11.37 MSAR, a saving of about 1.55 MSAR). The saving depended entirely on the unrealistic assumption above and was discarded by the project lead
(file `scripts/run_base_case.m`, earlier commit "Stage 4: MILP"). Time-sharing a drone's attention over the cells it serves was also rejected because it needs a footprint parameter for which no source exists.

**Keys.** `dji_dock3_specs` (charge time, flight time, speed); `Pd_drone` is no longer used in the optimization.
**Code.** `src/coverage_model.m` (detection uses DAS and camera only), `src/run_optimization.m`.

## D2. Two layers in Hot cells = detection (DAS) + verification (drone or camera over the whole cell)

**Decision.** A Hot cell will need two independent layers at every point of its boundary: x_i (DAS, the detection layer) and ρ_i (the verification layer). Neutral and Cold cells will need one layer.
A camera will count as the verification layer only if it covers the whole cell (f_i = 1), which does not happen because a camera site covers about 100 m.

**Why.** The operating sequence needs a detection and a verification step. The point-wise rule is strict on purpose: a Hot cell with a camera at one end is still unverified along the rest of its 1 km.

**Rejected.** (a) Counting every drone unit as a separate layer (`opts.hot_layer_counting = 'units'`, kept as an option): several drones of the same kind are not independent layers.
(b) A layer counted when it covers only a fraction of the cell: this would hide the uncovered points.
**Code.** `src/coverage_model.m` (`layers = x + [ver] + [f = 1]`).

## D3. Partial coverage of the cameras

**Decision.** A camera site will cover one stretch of Cov_CCTV = 100 m of boundary (table row `Cov_CCTV`, no written source), measured along the boundary. The detection of a cell will be the length-weighted mean of the
covered and uncovered parts (Eq. P_d,i = f_i p_C + x_i p_D (1 − f_i p_C)). The cameras of one site watch the same lanes, so the number of cameras per site will not multiply the covered length.

**Why.** A cell holds about 1 km of boundary and a camera covers about 100 m. A binary "a camera exists in the cell" would overstate the coverage by an order of magnitude; the real fraction is 1.9 % of the boundary (22 real interdiction crossings) to 2.6 % (32 sites).
Camera counts (16 per checkpoint, 4 per interdiction point) therefore do not change the result; a test shows that the covered length is identical when the counts change, and scales with the footprint parameter.

**Keys.** `alwatan_checkpoints` (six checkpoints), `okaz_shumaisi` (16 cameras at Al-Shumaisi), `ajel_32points` (32 interdiction points).
**Code.** `src/place_existing_assets.m`, `tests/test_cells_assets.m`.

## D4. Definition of AC₀

**Decision.** AC₀ will be a requirement on the DAS classifier itself: the check P_c,DAS ≥ AC₀ (0.983–0.993 ≥ 0.98, `wang2024`), performed once. It will not be a per-cell constraint.
The classification accuracy of the whole system, conditional on detection, will be computed for every cell and reported as a result (0.982–0.993).

**Why.** The project target concerns the quality of the classifier. A per-cell system-accuracy constraint forced DAS into every cell by definition, and the optimization then chose nothing (the result was circular).

**Rejected.** (a) AC₀ as a per-cell constraint of the whole-system accuracy (kept as `opts.ac0_definition = 'system'`). (b) Fusing classifications with 1 − ∏(1 − p): this assumes that the system knows which sensor is right, which it cannot.
With 1 − ∏(1 − p) the three sensors (0.9, 0.85, 0.85) would give 0.998; the weighted vote gives 0.952.

**Classification model.** Only sensors that detected can classify. They will vote with Bayes weights w_j = ln(p_j/(1 − p_j)) (binary, symmetric, independent errors); ties count one half. A ready verifying drone (accuracy 0.89, `khan_uav_thermal`, a proxy) adds a vote.
**Code.** `src/fuse_classification.m`, `src/coverage_model.m`, `tests/test_coverage_model.m`.

## D5. Elastic (soft) constraints for detection and layers

**Decision.** The detection constraint and the layer constraint will carry shortfall variables g_i, h_i, so the model will never be infeasible. The shortfalls will be reported for every cell.

**Why.** With P_d,DAS = 0.969 (`zeng2024`) and cameras covering 2.6 % of the boundary, P_d,i ≥ AD₀ = 0.98 is impossible in every cell (best value 0.975); with 3 drones the 36 Hot cells cannot all be verified. A hard model would simply report "infeasible".
The gap is itself a result: AD₀ will be reached only if the DAS reaches P_d,DAS ≥ 0.98 (sensitivity 0.98 and 1.0, `liu2016` reports 100 % in the field).
**Code.** `src/run_optimization.m` (stages 1 and 1b).

## D6. Lexicographic order of the objectives

**Decision.** The objectives will be minimised in this order: (1) the number of violating cells, (2) the total shortfall, (3) C_total, (4) maximise the sum of A_ver over the Hot cells.

**Why.** Requirements come first, the cost second. Drones have zero cost for us and DAS is needed almost everywhere, so after step 3 the drone positions would be arbitrary; step 4 places them where the Hot cells are verified most reliably without increasing C_total.

**Rejected.** A weighted sum with large penalties: the weights are arbitrary and numerically fragile (costs are about 10⁷ while shortfalls are about 10⁻²). A drone tie-break inside the cost objective (0.5 SAR per drone) was tried first and replaced by step 4.
**Code.** `src/run_optimization.m`.

## D7. Continuity and installability of the fibre

**Decision.** The boundary will be cut into 154 runs (maximal stretches inside one cell; 13 cells have more than one run). An interrogator will sit on a run and its two channels (one per direction) will serve contiguous runs, each carrying fibre,
up to R_DAS − L_lead = 20 km on each side. A DAS cell needs fibre on all its runs; fibre may exist in non-DAS cells only if it connects segments.

**Why.** The fibre is physically continuous and one FEBUS A1-NEO unit covers at most 2 R_DAS (two active channels, `N_ch_active`; the other two channels cannot add coverage because the lead-in counts against the 20 km range).
The cell numbers in `cells.csv` follow the grid scan, not the boundary, so the order along the boundary is rebuilt from the boundary polygon (`src/load_cells.m`).

**Rejected.** (a) Treating cells independently (unrealistic installation). (b) The equal-arc placement of DAS-map (it ignores which cells need DAS). (c) Interrogator sites only every fifth run (not needed: the problem stays tractable, 8,409 binary variables).
**Code.** `src/load_cells.m`, `src/run_optimization.m`, `tests/test_run_optimization.m` (independent continuity check).

## D8. Drone availability and the verification layer

**Decision.** A station with m drones will have a ready drone with probability A(m) = min(1, m·T_drone/(T_drone + T_charge)) (0.635 for one drone, 1.0 for two). A cell is ready with
A_ver = 1 − ∏(1 − A(n_s)) over the stations that serve it, and it has the verification layer if A_ver ≥ A_ver,min = 0.9 (PLACEHOLDER; sensitivity 0.8 and 0.95).

**Why.** "Drone ready" is the property that matters for verification; ignoring availability would overstate the layer. With 0.9, a station needs two drones (or three single-drone stations).
**Alternative rejected.** Availability 1 for any drone: overstates the layer by 37 % for a single drone.
**Keys.** `dji_dock3_specs`, table rows `T_drone`, `T_charge`, `A_ver_min`. **Code.** `src/coverage_model.m`.

## D9. Response reach

**Decision.** A station will serve a cell if the farthest point of the cell is within R_resp = min(Cov_drone, v_drone (T_resp − T_launch)) = min(11 km, 15 m/s × 290 s) = 4,350 m (T_resp = 300 s PLACEHOLDER; sensitivity 180 and 600 s gives 2,550 m and 8,850 m).
v_drone = 15 m/s is the Normal-mode maximum of the Matrice 4D/4TD, the only mode allowed with the Dock 3 (`dji_dock3_specs`). T_launch = 10 s comes from a reseller page (`advexure_dock3`, VERIFY on the manufacturer page).

**Why.** The verification must arrive within the response time; the 11 km operating radius is a range, not a response distance.
**Code.** `src/load_params.m`, `src/coverage_model.m`, `src/drone_layout.m`.

## D10. Existing assets: locations, counts and cost

**Decision.** The DAS-map figures (16 cameras, 3 drones, 5 patrols) will not be used because they are illustrative. The six entrance checkpoints (`alwatan_checkpoints`) will be matched to their roads in OpenStreetMap (name, ref, geometry) and placed where the road crosses the boundary:
Al-Shumaisi (new): road 40 west; Al-Shumaisi old (Hudaybiyah): Old Makkah Road (ref 290); Al-Sharaye: road 40 east (identification by ref continuity and project-lead information; OSM has no "Sail" name); Al-Hada: Kara–Hada road (ref 15, east); Al-Nawariyah: Medina road (ref 15).
Al-Laith is excluded because it is far from the boundary (project lead). The 32 interdiction points (`ajel_32points`) are assumed at crossings of lesser roads (22 real crossings available, 10 filler sites in the 32-site case); camera counts are 16 per checkpoint (assumed from `okaz_shumaisi`) and 4 per interdiction point (PLACEHOLDER).
Existing assets will have zero capital cost for us. Patrols (5/15/30) will be a response resource and will not enter the detection constraint (no accuracy values exist).

**Result.** The choice between 22 real crossings and 32 sites does not change any result (identical C_total, DAS cells and Hot verification).
**Code.** `src/place_existing_assets.m`, `report/observations.md`.

## D11. Candidate drone stations and exactness

**Decision.** Candidate stations will be boundary points every 500 m and an interior grid every 2 km (457 candidates; real dock sites are not public). Stations whose response reach is contained in that of another candidate will be removed (95 remain); this is exact because a station can serve any number of cells.
All steps will be solved with `intlinprog` to proven optimality (gap 0 %); one case takes about 10 s (at most 105 s over 47 cases).
**Rejected.** A heuristic placement (no optimality proof). **Code.** `src/run_optimization.m`.

## D12. Baselines

**Decision.** (1) Existing assets only (no DAS). (2) The same number of drones spread evenly on the boundary (pairs of drones per station). (2b) The same drones spread evenly over the same candidate stations used by the optimizer (k-means on all candidates, snapped).
(2c) Even boundary spacing at the best of 40 phase offsets, i.e. the strongest even layout that does not optimise positions.
**Why.** 2 is the naive layout; 2b makes the comparison fair with respect to the candidate set; 2c shows that even the best regular layout is below the optimum.
**Code.** `scripts/make_stage5.m`.

## D13. Cost accounting

**Decision.** C_total = (C_DAS,int + C_shelter) × (number of interrogators) + C_fibre × (fibre length). The installation cost is inside C_fibre (table row `C_fiber`, "with installation").
C_shelter = 0 is a PLACEHOLDER (no price found): C_total will therefore be lower than the real additional cost and this will be stated wherever C_total appears. C_fibre = 60,700 SAR/km has no Saudi source and will be varied ×0.5 and ×2.
Drone and camera prices (`C_drone` = 148,887 SAR) are used only for the reference cost of closing the verification gap.

## D14. Other choices

* **Independence.** The sensors will be assumed to fail independently (stated as an assumption; it may be optimistic because of shared weather and terrain).
* **Point-Sensor Networks.** They are present at the checkpoints together with the cameras, but all their parameters are TBD, so they will be excluded from this version (switch `use_PS`).
* **Scenarios.** The number of drones is not published (`yenisafak2026drones` only states that drones are used against unauthorized entry); three scenarios will be run: 3 / 6 / 12 drones (and 5 / 15 / 30 patrols), all ASSUMED.
* **Power constraint.** (P_DAS + P_shelter) × (interrogators) < P₀ = 29,100 W (the power of a CCTV-only system covering the same boundary: 1,455 × 20 W). It will not be binding (1,360 W, 4.7 % of P₀) and this will be reported.
