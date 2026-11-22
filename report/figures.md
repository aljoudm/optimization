# Figures and tables (reference draft for Section 3 and the paper)

Each entry: file, proposed label, proposed caption (English, future tense where it describes the proposal), and what it shows.
All maps have the style of DAS-map (zone colours: Cold blue, Neutral yellow, Hot red). The cabin warning is printed in every map: C_total does not include the cabins (C_shelter = 0, PLACEHOLDER).

## Figures

| # | File | Label | Proposed caption | What it shows |
| --- | --- | --- | --- | --- |
| 1 | `report/s3_optimization_map.pdf` (copy of `results/map_medium.pdf`) | `fig:s3-map` | "Optimal allocation for the medium scenario (6 drones, $P_{d,\mathrm{DAS}}=0.969$, $T_{\mathrm{resp}}=300$ s). The DAS fibre will cover the whole boundary with four interrogators; three drone stations (two drones each) will give 22 of the 36 Hot cells a verification layer. $C_{\mathrm{total}}$ excludes the cost of the cabins." | The final allocation: zones, DAS fibre, interrogators, drone stations with their response circles ($R_{\mathrm{resp}}=4.35$ km), Hot cells with and without verification. |
| 2 | `results/map_low.pdf`, `results/map_low.png` | `fig:map-low` | "Optimal allocation for the low scenario (3 drones): 9 of 36 Hot cells will be verified." | Same as Fig. 1 for 3 drones (two stations). |
| 3 | `results/map_high.pdf`, `results/map_high.png` | `fig:map-high` | "Optimal allocation for the high scenario (12 drones): 34 of 36 Hot cells will be verified." | Same as Fig. 1 for 12 drones (six stations). |
| 4 | `results/map_baseline2_medium.pdf` (and `_low`, `_high`) | `fig:map-baseline2` | "Baseline without optimization: the same six drones spread evenly along the boundary verify only 7 of 36 Hot cells." | The naive layout, for comparison with Fig. 1. |
| 5 | `results/map_baseline2b_medium.pdf` (and `_low`, `_high`) | `fig:map-baseline2b` | "Baseline 2b: the same drones spread evenly over the same candidate stations that the optimizer uses verify only 2 of 36 Hot cells (medium)." | A fair comparison with respect to the candidate set. |
| 6 | to be drawn | `fig:opt-flowchart` | "Flowchart of the optimization procedure." | The algorithm steps of `report/optimization_formulation.tex` (Section "Algorithm steps"). |
| 7 | DAS-map `results_zones/zone_cells_map.png` (not copied here; belongs to the DAS-map repository) | `fig:zones` | "Hot, Neutral and Cold cells of the Makkah boundary (139 cells of 1 km)." | The zoning that the optimization takes as input. |
| 8 | (to be produced if wanted) bar chart of Hot cells verified: optimal, 2c, 2b, even boundary, for 3 / 6 / 12 drones, from `results/baseline2_even_drones.csv` | `fig:baseline-bars` | "Hot cells with a verification layer: optimal placement against evenly spread drones." | The benefit of the optimization at a glance. |

## Tables

| # | File | Label | Proposed caption | Content |
| --- | --- | --- | --- | --- |
| T1 | `report/table_summary.tex` (from `results/report_summary.csv`) | `tab:summary` | "Summary of the optimal allocations (scenarios and $P_{d,\mathrm{DAS}}$). $C_{\mathrm{total}}$ excludes the cabins ($C_{\mathrm{shelter}}=0$, PLACEHOLDER)." | $C_{\mathrm{total}}$, interrogators, drones, Hot verified, detection (minimum $P_d$ and gap), power against $P_0$. The CSV also has the classification columns (classifier check, whole-system accuracy). |
| T2 | `results/baseline2_even_drones.csv` | `tab:baseline2` | "Hot cells with a verification layer: optimal placement against even placements." | Optimal 9 / 22 / 34; even boundary 0 / 7 / 12; 2b 0 / 2 / 22; 2c 9 / 16 / 25 (of 36). |
| T3 | `results/baseline1_existing_assets_only.csv` | `tab:baseline1` | "Detection with the existing assets only (cameras, no DAS)." | Boundary covered (1.9 %), mean detection 0.02, maximum 0.19, no cell reaches $\mathrm{AD}_0$. |
| T4 | `results/gap_closing_cost.csv` | `tab:gap-cost` | "Drones needed to verify every Hot cell and the cost of the extra drones, for three response times." | Fewest drones 24 / 14 / 8; extra drones and cost per scenario. |
| T5 | `report/optimization_formulation.tex` (longtable) | `tab:symbols` | "Symbols, values and source keys." | Every symbol with its value and source key or tag. |
| T6 | `results/cells_medium.csv` (also `_low`, `_high`) | (appendix) | "Per-cell results of the medium scenario." | Zone, DAS, detection, gap, $A_{\mathrm{ver}}$, layers, system accuracy, nearest drone station and distance. |
| T7 | `results/summary_runs.csv` | (appendix) | "All 47 optimization runs." | Scenarios, sensitivity cases, solve time and gap. |
