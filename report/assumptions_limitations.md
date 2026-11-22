# Assumptions, placeholders and limitations (reference draft for Section 3 and the paper)

Tags follow `load_params.m`: TABLE = value from `parameters_reviewed.tsv`; ASSUMED = assumption stated by the project; PLACEHOLDER = no source, a value kept in one place and varied; PROXY = a published number for a similar but not identical quantity;
SOURCE (VERIFY) = a number whose source must be checked. The effect column says how the result will change if the value is wrong.

## A. Assumptions and placeholders

| # | Item | Value / tag | Key | Effect on the result |
| --- | --- | --- | --- | --- |
| 1 | P_d,DAS (DAS detection probability) | 0.969, literature value (one disturbance site) until the Monte Carlo simulation of the team provides it; sensitivity 0.98 and 1.0 | `zeng2024`, `liu2016` | Decides whether AD₀ = 0.98 is met: 0.969 leaves a gap of at most 0.011 in all 139 cells; 0.98 or higher removes it. Allocation and C_total do not change. |
| 2 | P_c,DAS (DAS classification accuracy) | 0.983–0.993, literature range; both ends run | `wang2024` (via a review, Shao 2025) | Only the whole-system accuracy reported (0.982 to 0.992). The classifier check P_c,DAS ≥ AC₀ passes at both ends. |
| 3 | P_c,drone | 0.89, PROXY: precision (not accuracy) of YOLOv8s on the HIT-UAV thermal dataset, no animals | `khan_uav_thermal` | Only the reported whole-system accuracy (the drone adds one vote). |
| 4 | T_charge, T_drone, v_drone | 27 min, 47 min, 15 m/s; DJI figures only | `dji_dock3_specs` | Drone ready probability (0.635 / 1.0) and R_resp; other drone models will differ. |
| 5 | T_launch | 10 s, SOURCE (reseller page), VERIFY on the manufacturer page | `advexure_dock3` | R_resp changes by 150 m per 10 s; small. |
| 6 | T_resp | 300 s, PLACEHOLDER, tied to T_alert (TBD, Student 1); sensitivity 180 and 600 s | table row `T_resp` | Strong: Hot cells verified (medium) 15 / 22 / 32 and fewest drones 24 / 14 / 8 for 180 / 300 / 600 s. |
| 7 | A_ver,min | 0.9, PLACEHOLDER; sensitivity 0.8 and 0.95 | table row `A_ver_min` | No change in the medium scenario; a lower value would let single-drone stations count (0.635 is below 0.8). |
| 8 | C_shelter | 0, PLACEHOLDER (no price found) | table row `C_shelter` | **C_total is under-stated** by the cost of 4 cabins; the figure must carry this note. |
| 9 | C_fibre | 60,700 SAR/km, PLACEHOLDER (no published Saudi source; flagged as very doubtful); sensitivity ×0.5 and ×2 | table row `C_fiber` | C_total 6.96 / 11.37 / 20.20 MSAR; fibre is 63 % to 87 % of C_total, so this is the most important number that still needs a source. |
| 10 | C_DAS,int | 636,000 SAR = 150,000 EUR × 4.24 (exchange rate assumed, 15 Oct 2026) | table row `C_DAS_int` | 22 % of C_total; a different rate changes it proportionally. |
| 11 | P_shelter | 170 W, upper bound of 73 to 170 W (cooling is 30 to 50 % of the site load) | `gsma_coolsure` | Power only; far below P₀ in any case. |
| 12 | P₀ | 29,100 W = CCTV-only system covering the boundary (1,455 × 20 W), a proposal of the table | table row `P0` | The power constraint is not binding (4.7 %); it would bind only if P_shelter were above about 7 kW. |
| 13 | Number of drones | 3 / 6 / 12, ASSUMED scenarios (the number is not published); drones are used against unauthorized entry | `yenisafak2026drones` | The Hot verification (9 / 22 / 34 of 36) depends directly on it; the cost and the DAS do not. |
| 14 | Patrols | 5 / 15 / 30, ASSUMED; response resource only | (no key) | None: patrols do not enter any constraint. |
| 15 | Camera positions | Assumed at the road crossings of the six checkpoints (matched to OSM roads) and of 32 interdiction points; real locations are not public | `alwatan_checkpoints`, `ajel_32points` | Cameras cover 1.9 to 2.6 % of the boundary; any placement gives a similar small share, and 22 versus 32 sites changes nothing. |
| 16 | Cameras per site | 16 per checkpoint (assumed from the 16 cameras of Al-Shumaisi), 4 per interdiction point (PLACEHOLDER) | `okaz_shumaisi` | None on the allocation: all cameras of a site watch the same lanes, so the covered length does not depend on the count. |
| 17 | Camera footprint | 100 m of boundary per site (`Cov_CCTV`, no written source in the table) | table row `Cov_CCTV` | Covered boundary scales with it (×4 gives 7.6 %); still far from AD₀. |
| 18 | Checkpoint matching | Al-Sharaye = road 40 east of Makkah (identification by ref continuity and the project lead; OSM has no "Sail" name, medium confidence); Al-Hada real distance from the boundary not verified; Al-Laith excluded | (no key) | Changes the positions of up to two camera sites; no effect on the result. |
| 19 | Candidate drone stations | boundary every 500 m + interior grid every 2 km; real docks not public | — | The Hot verification of the optimum is a bound for these candidates. |
| 20 | Q (stations serving one cell) | 2; tested with Q = 3 (which also allows three single-drone stations, ready probability 0.951 >= 0.9) | — | None: Q = 3 gives identical results in all three scenarios (C_total, Hot verified 9 / 22 / 34, drones used, fewest drones 14, sum of A_ver); the case needs more drones than a two-drone station for the same reach. Q = 3 takes 3 to 4 times longer to solve. |
| 21 | Independence of sensors | assumed everywhere | — | May overstate detection and classification when sensors share weather and terrain (optimistic). |
| 22 | Intruder position | uniform along the boundary of a cell | — | A non-uniform (e.g. wadi-biased) position would lower the camera benefit further. |
| 23 | Zone classification | k-means on cell risk; the weights (slope, road distance) are assumed until the Risk Matrix of Student 3 exists; cell size 1 km; Arafat boundary provisional | DAS-map | Changes which cells are Hot (and therefore the number to verify). |
| 24 | L_lead | 0 m (cabin on the boundary) | table row `L_leadin` | A lead-in reduces the reach of every channel and could require more units. |
| 25 | lambda_cold, FAR | 1–2 events/day, 2 % (1 − precision) | table rows | Not used by the optimization; simulation inputs. |

## B. Limits of the model

1. **Drones have a unit watch over the whole cell and every cell in reach.** A station will be assumed to serve all the cells within R_resp; simultaneous alarms in different cells are not modelled, so the Hot verification is an upper bound for a given number of drones.
2. **Existing assets are assumed, not surveyed.** Real camera, drone and Point-Sensor Network locations and numbers are not public; all conclusions about them hold for the assumed layout. The positions will be replaced when real data are available (one parameter file).
3. **C_total is incomplete.** The cabins are not costed (C_shelter = 0), the fibre price has no source, and the interrogator price rests on an assumed exchange rate. The cost of civil works along the boundary (permits, terrain) is inside the single C_fibre figure.
4. **The DAS detection probability comes from the literature.** It will be replaced by the result of the Monte Carlo simulation of the team; until then AD₀ is reported as met or missed for three values of P_d,DAS.
5. **Detection is not tested against false alarms.** The false-alarm target (FAR = 2 %, 1 − precision) is not part of the optimization; a lower detection threshold that increases P_d,DAS would also increase false alarms.
6. **Classification is binary.** Human versus non-human, symmetric and independent errors; animals and vehicles on roads are not modelled separately (the Cold zones have no simulated intrusions in the digital twin until lambda_cold > 0).
7. **Static, single time period.** Seasonal and daily variation (Hajj crowds, temperature, wind) and the time to move drones between stations are not modelled.
8. **Fibre route.** The fibre will be assumed to follow the boundary polygon exactly (no slack factor); the boundary length used is 145,461 m (planar), 0.008 % below the table value, and the Arafat part of the boundary is provisional.
9. **Interrogator sites.** They are restricted to the middle of boundary runs; this discretisation can cost at most one run length in a channel.
10. **Solver licence.** The runs were made with a trial licence of the Optimization Toolbox; the results do not depend on it, but the toolbox must be available to reproduce them.
11. **Table items still open.** `T_alert`, `L_loc` and the Point-Sensor Networks values are TBD; the model has switches for them and does not use them now.
