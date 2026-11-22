function make_stage5()
% MAKE_STAGE5  Stage 5 outputs from results/allocs.mat and results/summary_runs.csv (run run_all_cases first):
%   per-cell CSVs, allocation maps (one per scenario + the evenly-spread-drone baseline), baseline 1 (existing assets only),
%   baseline 2 (same number of drones spread evenly along the boundary, no optimization), the cost of closing the verification gap,
%   and the summary table for the report.
% WARNING printed everywhere: C_total has NO cabin cost (C_shelter = 0, a PLACEHOLDER); it is lower than the real additional cost.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
outDir = fullfile(here, '..', 'results');   repDir = fullfile(here, '..', 'report');
if ~exist(repDir, 'dir'), mkdir(repDir); end
P = load_params();   P.assets.allow_filler = false;   v = P.v;   cells = load_cells(P);   A = place_existing_assets(P, cells);
S = load(fullfile(outDir, 'allocs.mat'), 'allocs');   allocs = S.allocs;
T = readtable(fullfile(outDir, 'summary_runs.csv'), 'TextType', 'string');   T.Properties.VariableNames{1} = 'kind';
scen = P.assets.scenarios;   ring = cells.ring;   L = ring.L;   hot = cells.zoneCode == 3;   zn = {'Cold', 'Neutral', 'Hot'};
CABIN = 'C_total does NOT include cabin cost: C_shelter = 0 (PLACEHOLDER, no price found), so the real additional cost is higher.';
fprintf('\n*** %s ***\n', CABIN);
pick = @(sc, pd) allocs{find(cellfun(@(a) strcmp(a.caseInfo.label, 'grid') && strcmp(a.caseInfo.interdiction, 'real22') && strcmp(a.scenario.name, sc) && abs(a.caseInfo.Pd_DAS - pd) < 1e-9 ...
    && abs(a.caseInfo.Pc_DAS - 0.983) < 1e-9 && a.caseInfo.T_resp == 300 && a.caseInfo.A_ver_min == 0.9 && a.caseInfo.C_fiber_factor == 1, allocs), 1)};

%% ---- (a) per-cell CSVs + (b) maps for the optimal solutions, and baseline 2 ------------------------------------------------------------------
sumRows = {};   b2Rows = {};
for s = 1:numel(scen)
    nm = scen(s).name;   N = scen(s).N_drone;   al = pick(nm, 0.969);
    lay = drone_layout(cells, P, al.drones.station_xy, al.drones.n(:));
    assert(all(lay.verified(hot) >= al.cells.verified(hot)), 'actual verification must not be below the MILP state');
    hv = nnz(lay.verified & hot);
    if hv ~= al.hot_verified, fprintf('note: %s: Hot verified %d from the actual station layout vs %d in the MILP state (the layout is what counts)\n', nm, hv, al.hot_verified); end
    Tc = al.cells;
    layers = double(Tc.DAS) + double(lay.verified) + double(A.coverFrac >= 1 - 1e-12);
    csv = table((1:cells.n)', string(cells.zone), cells.L_m, Tc.DAS, A.coverFrac, Tc.Pd, Tc.Pd_gap, lay.A_ver, lay.verified, layers, Tc.layers_needed, layers >= Tc.layers_needed, ...
        Tc.Pc_system, Tc.Pc_system >= v.AC0 - 1e-12, lay.nearest, lay.nearest_dist_m, lay.nearest_in_reach, Tc.violates | ~(layers >= Tc.layers_needed), Tc.fibre_cost, ...
        'VariableNames', {'cell_id','zone','boundary_length_m','DAS','camera_fraction','Pd','Pd_gap_to_AD0','A_ver','verification_layer','layers','layers_needed', ...
        'two_layers_ok_or_one_layer_ok','Pc_system','Pc_system_ge_AC0','nearest_drone_station','nearest_station_distance_m','nearest_station_within_R_resp','below_a_requirement','fibre_cost_SAR'});
    writetable(csv, fullfile(outDir, sprintf('cells_%s.csv', nm)), 'Encoding', 'UTF-8');
    % optimal map
    f = figure('Color', 'w', 'Position', [60 40 980 760], 'Visible', 'off');   ax = axes(f);
    draw_allocation_map(ax, cells, P, al, lay, al.drones.station_xy, al.drones.n(:), sprintf('Optimal allocation, scenario %s (%d drones): C_{total} %.2f MSAR, %d interrogators, DAS %d/%d, Hot verified %d/%d', ...
        nm, N, al.C_total / 1e6, al.n_interrogators, al.cells_with_DAS, cells.n, hv, nnz(hot)), CABIN);
    exportgraphics(f, fullfile(outDir, sprintf('map_%s.png', nm)), 'Resolution', 250);   exportgraphics(f, fullfile(outDir, sprintf('map_%s.pdf', nm)), 'ContentType', 'vector');   close(f);
    % baseline 2: the same number of drones spread evenly along the boundary (no optimization). Pairs (2 drones per station, ready prob 1) with one single if N is odd.
    Ns = ceil(N / 2);   nd2 = [2 * ones(1, floor(N / 2)), mod(N, 2) * ones(1, mod(N, 2))];   sArc = ((1:Ns) - 0.5) * L / Ns;   xy2 = zeros(Ns, 2);
    for j = 1:Ns, [~, k] = min(abs(ring.s - sArc(j)));   xy2(j, :) = [ring.x(k) ring.y(k)]; end
    lay2 = drone_layout(cells, P, xy2, nd2(:));
    % variant: N single-drone stations evenly spread
    sA1 = ((1:N) - 0.5) * L / N;   xy1 = zeros(N, 2);   for j = 1:N, [~, k] = min(abs(ring.s - sA1(j)));   xy1(j, :) = [ring.x(k) ring.y(k)]; end
    lay1 = drone_layout(cells, P, xy1, ones(N, 1));
    f = figure('Color', 'w', 'Position', [60 40 980 760], 'Visible', 'off');   ax = axes(f);
    draw_allocation_map(ax, cells, P, al, lay2, xy2, nd2(:), sprintf('Baseline 2 (no optimization), %s: same %d drones spread evenly; Hot verified %d/%d', nm, N, nnz(lay2.verified & hot), nnz(hot)), CABIN);
    exportgraphics(f, fullfile(outDir, sprintf('map_baseline2_%s.png', nm)), 'Resolution', 250);   exportgraphics(f, fullfile(outDir, sprintf('map_baseline2_%s.pdf', nm)), 'ContentType', 'vector');   close(f);
    b2Rows(end+1, :) = {string(nm), N, hv, nnz(lay2.verified & hot), nnz(lay1.verified & hot), nnz(hot), 100 * hv / nnz(hot), 100 * nnz(lay2.verified & hot) / nnz(hot), ...
        nnz(lay.verified), nnz(lay2.verified), cells.n}; %#ok<AGROW>
    % summary rows (Pd_DAS 0.969 and 0.98)
    for pd = [0.969 0.98]
        a = pick(nm, pd);   hvv = nnz(drone_layout(cells, P, a.drones.station_xy, a.drones.n(:)).verified & hot);
        sumRows(end+1, :) = {string(nm), N, pd, a.C_total, a.cost.fibre, a.fibre_share, a.n_interrogators, a.fibre_km, a.drones.total, a.drones.stations, hvv, nnz(hot), ...
            min(a.cells.Pd), mean(a.cells.Pd), a.detection_gap.max, a.detection_gap.cells_below_AD0, v.Pc_DAS, a.classifier_check.ok, min(a.cells.Pc_system), a.system_accuracy_ok, ...
            a.power.total_W, v.P0, 100 * a.power.total_W / v.P0, string(CABIN)}; %#ok<AGROW>
    end
end

%% ---- baseline 1: existing assets only (cameras, no DAS) ------------------------------------------------------------------------------------------
Pd0 = A.coverFrac * v.Pd_CCTV;
rows1 = {};
for k = [3 2 1]
    in = cells.zoneCode == k;
    rows1(end+1, :) = {string(zn{k}), nnz(in), 100 * sum(A.coveredLen_m(in)) / sum(cells.L_m(in)), min(Pd0(in)), mean(Pd0(in)), max(Pd0(in)), nnz(Pd0(in) >= v.AD0)}; %#ok<AGROW>
end
rows1(end+1, :) = {"All", cells.n, 100 * sum(A.coveredLen_m) / L, min(Pd0), mean(Pd0), max(Pd0), nnz(Pd0 >= v.AD0)};
B1 = cell2table(rows1, 'VariableNames', {'zone','cells','boundary_covered_by_cameras_pct','Pd_min','Pd_mean','Pd_max','cells_meeting_AD0'});
writetable(B1, fullfile(outDir, 'baseline1_existing_assets_only.csv'));
writetable(table((1:cells.n)', string(cells.zone), A.coverFrac, Pd0, v.AD0 - Pd0, 'VariableNames', {'cell_id','zone','camera_fraction','Pd_existing_only','gap_to_AD0'}), fullfile(outDir, 'baseline1_existing_assets_only_per_cell.csv'));
fprintf('\n== Baseline 1: existing assets only (cameras, no DAS, drones do not detect) ==\n');   disp(B1);

%% ---- baseline 2 table ---------------------------------------------------------------------------------------------------------------------------
B2 = cell2table(b2Rows, 'VariableNames', {'scenario','N_drone','Hot_verified_optimal','Hot_verified_even_pairs','Hot_verified_even_singles','Hot_cells','optimal_pct','even_pairs_pct','cells_verified_optimal','cells_verified_even_pairs','cells'});
writetable(B2, fullfile(outDir, 'baseline2_even_drones.csv'));
fprintf('\n== Baseline 2: same drones spread evenly (no optimization) vs the optimal placement, Hot cells with the verification layer ==\n');   disp(B2);
fprintf('(even_pairs: 2 drones per station spaced evenly by arc length, one single drone if N is odd; even_singles: one drone per station, ready prob 0.635 < A_ver_min)\n');

%% ---- (d) cost of closing the verification gap ----------------------------------------------------------------------------------------------------
G = {};   Cd = v.C_drone_ref;
for s = 1:numel(scen)
    for tr = [180 300 600]
        r = T(T.interdiction_case == "real22" & T.scenario == string(scen(s).name) & T.Pd_DAS == 0.969 & T.A_ver_min == 0.9 & T.T_resp_s == tr & (T.kind == "grid" | T.kind == "T_resp"), :);
        need = r.fewest_drones_all_Hot_verified(1);   extra = max(0, need - scen(s).N_drone);
        G(end+1, :) = {string(scen(s).name), scen(s).N_drone, tr, min(11000, v.v_drone_ms * (tr - v.T_launch_s)), r.Hot_verified(1), need, extra, extra * Cd, Cd}; %#ok<AGROW>
    end
end
Gt = cell2table(G, 'VariableNames', {'scenario','N_drone','T_resp_s','R_resp_m','Hot_verified','drones_needed_for_all_Hot','extra_drones','extra_cost_SAR','C_drone_SAR'});
writetable(Gt, fullfile(outDir, 'gap_closing_cost.csv'));
fprintf('\n== Cost of closing the verification gap (extra drones x C_drone = %.0f SAR; drones exist already, so this is a reference figure) ==\n', Cd);   disp(Gt);

%% ---- (e) summary table for the report ---------------------------------------------------------------------------------------------------------------
Sm = cell2table(sumRows, 'VariableNames', {'scenario','N_drone','Pd_DAS','C_total_SAR','fibre_cost_SAR','fibre_share','interrogators','fibre_km','drones_used','drone_stations','Hot_verified','Hot_cells', ...
    'Pd_min','Pd_mean','Pd_max_gap_to_AD0','cells_below_AD0','Pc_DAS','classifier_check_Pc_DAS_ge_AC0','system_accuracy_min','cells_system_accuracy_ge_AC0','power_W','P0_W','power_pct_of_P0','note'});
writetable(Sm, fullfile(outDir, 'report_summary.csv'), 'Encoding', 'UTF-8');
fprintf('\n== Summary for the report (scenario x Pd_DAS) ==\n');   disp(Sm(:, 1:end-1));
fid = fopen(fullfile(repDir, 'table_summary.tex'), 'w');
fprintf(fid, '%% Generated by scripts/make_stage5.m. C_total has no cabin cost (C_shelter = 0, PLACEHOLDER).\n\\begin{tabular}{lrrrrrrr}\n\\hline\n');
fprintf(fid, 'Scenario & $P_{d,\\mathrm{DAS}}$ & $C_{\\mathrm{total}}$ (MSAR) & Interrog. & Drones & Hot verified & Min $P_d$ / gap & Power / $P_0$ \\\\\n\\hline\n');
for r = 1:height(Sm)
    fprintf(fid, '%s (%d) & %.3f & %.2f & %d & %d & %d/%d & %.3f / %.3f & %.0f / %.0f W \\\\\n', Sm.scenario(r), Sm.N_drone(r), Sm.Pd_DAS(r), Sm.C_total_SAR(r) / 1e6, Sm.interrogators(r), Sm.drones_used(r), Sm.Hot_verified(r), Sm.Hot_cells(r), Sm.Pd_min(r), Sm.Pd_max_gap_to_AD0(r), Sm.power_W(r), Sm.P0_W(r));
end
fprintf(fid, '\\hline\n\\end{tabular}\n');   fclose(fid);
fprintf('\nFiles written to %s (cells_*.csv, map_*.png/pdf, map_baseline2_*.png/pdf, baseline1*.csv, baseline2*.csv, gap_closing_cost.csv, report_summary.csv) and %s\n', outDir, fullfile(repDir, 'table_summary.tex'));
fprintf('*** %s ***\n', CABIN);
end
