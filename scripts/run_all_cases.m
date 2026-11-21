function T = run_all_cases(timeLimit)
% RUN_ALL_CASES  Stage 4 driver (model of 22 Nov: DAS + cameras detect, drones verify).
%   scenarios (drones 3 / 6 / 12, patrols 5 / 15 / 30)  x  Pd_DAS (0.969 base, 0.98 = AD0, 1.0)  x  interdiction case (22 real / 32 with filler)
%   + Pc_DAS = 0.993 (upper end of wang2024) on the base case
%   + sensitivity: T_resp 180 / 600 s, A_ver_min 0.8 / 0.95, C_fiber x0.5 / x2 (medium scenario)
% Pc_DAS only changes the whole-system accuracy (a result), not the allocation, so it is evaluated on the base case only.
% timeLimit = seconds per case (default 1800). Results: results/summary_runs.csv, results/allocs.mat
if nargin < 1, timeLimit = 1800; end
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
outDir = fullfile(here, '..', 'results');   if ~exist(outDir, 'dir'), mkdir(outDir); end
P0 = load_params();   cells = load_cells(P0);
fprintf('Optimization Toolbox: %s\n', tern(license('test', 'Optimization_Toolbox') && exist('intlinprog', 'file') > 0, 'available', 'MISSING'));
interd = struct('name', {'real22', 'with_filler32'}, 'allow_filler', {false, true});
scen = P0.assets.scenarios;
C = struct('label', {}, 'inter', {}, 'sc', {}, 'pd', {}, 'pc', {}, 'cfib', {}, 'tresp', {}, 'aver', {});
for a = 1:2, for s = 1:3, for pd = [0.969 0.98 1.0]
    C(end+1) = struct('label', 'grid', 'inter', a, 'sc', s, 'pd', pd, 'pc', 0.983, 'cfib', 1, 'tresp', 300, 'aver', 0.9); %#ok<AGROW>
end, end, end
for s = 1:3, C(end+1) = struct('label', 'Pc_DAS 0.993', 'inter', 1, 'sc', s, 'pd', 0.969, 'pc', 0.993, 'cfib', 1, 'tresp', 300, 'aver', 0.9); end %#ok<AGROW>
for pd = [0.969 0.98]
    for s = 1:3
        for tr = [180 600], C(end+1) = struct('label', 'T_resp', 'inter', 1, 'sc', s, 'pd', pd, 'pc', 0.983, 'cfib', 1, 'tresp', tr, 'aver', 0.9); end %#ok<AGROW>
        for av = [0.8 0.95], C(end+1) = struct('label', 'A_ver_min', 'inter', 1, 'sc', s, 'pd', pd, 'pc', 0.983, 'cfib', 1, 'tresp', 300, 'aver', av); end %#ok<AGROW>
    end
end
for f = [0.5 2], C(end+1) = struct('label', 'C_fiber', 'inter', 1, 'sc', 2, 'pd', 0.969, 'pc', 0.983, 'cfib', f, 'tresp', 300, 'aver', 0.9); end %#ok<AGROW>
rows = {};   allocs = {};   t0 = tic;   reachCache = [];   lastInter = 0;
for c = 1:numel(C)
    cs = C(c);   P = P0;
    P.assets.allow_filler = interd(cs.inter).allow_filler;   P.v.Pd_DAS = cs.pd;   P.v.Pc_DAS = cs.pc;   P.v.C_fiber_per_km = P0.v.C_fiber_per_km * cs.cfib;
    P.v.T_resp_s = cs.tresp;   P.v.A_ver_min = cs.aver;   P.v.R_resp_m = min(P.v.Cov_drone_m, P.v.v_drone_ms * (P.v.T_resp_s - P.v.T_launch_s));
    if cs.inter ~= lastInter, A = place_existing_assets(P, cells);   lastInter = cs.inter; end
    M = coverage_model(P, cells, A, reachCache);   reachCache = struct('reach', M.reach, 'stationXY', M.stationXY, 'radius', M.reach_radius);
    fprintf('\n--- case %d/%d [%s]: %s | scenario %s (%d drones) | Pd_DAS %.3f | Pc_DAS %.3f | T_resp %g s (R_resp %.0f m) | A_ver_min %.2f | C_fiber x%g | elapsed %.0f s\n', ...
        c, numel(C), cs.label, interd(cs.inter).name, scen(cs.sc).name, scen(cs.sc).N_drone, cs.pd, cs.pc, cs.tresp, P.v.R_resp_m, cs.aver, cs.cfib, toc(t0));
    al = run_optimization(cells, P, 'Scenario', scen(cs.sc), 'Assets', A, 'Model', M, 'Verbose', true, 'TimeLimit', timeLimit);
    al.caseInfo = struct('label', cs.label, 'interdiction', interd(cs.inter).name, 'Pd_DAS', cs.pd, 'Pc_DAS', cs.pc, 'C_fiber_factor', cs.cfib, 'T_resp', cs.tresp, 'A_ver_min', cs.aver);
    allocs{end+1} = al; %#ok<AGROW>
    hot = cells.zoneCode == 3;
    rows(end+1, :) = {string(cs.label), string(interd(cs.inter).name), nnz([A.sites.in]), string(scen(cs.sc).name), scen(cs.sc).N_drone, cs.pd, cs.pc, cs.tresp, cs.aver, cs.cfib, ...
        al.C_total, al.fibre_share, al.n_interrogators, al.fibre_km, al.cells_with_DAS, al.drones.total, al.drones.stations, nnz(al.cells.verified & hot), nnz(hot), ...
        al.n_violations, nnz(al.cells.Pd < P.v.AD0 - 1e-12), al.detection_gap.max, al.A_ver_hot.sum, al.min_drones_no_gap, al.system_accuracy_ok, min(al.cells.Pc_system), al.power.total_W, al.solver.stage2.relativegap, al.elapsed_s}; %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'kind','interdiction_case','camera_sites','scenario','N_drone','Pd_DAS','Pc_DAS','T_resp_s','A_ver_min','C_fiber_factor', ...
    'C_total_SAR','fibre_share','interrogators','fibre_km','DAS_cells','drones_used','drone_stations','Hot_verified','Hot_cells', ...
    'cells_below_requirement','cells_below_AD0','max_Pd_gap','sumA_ver_Hot','fewest_drones_all_Hot_verified','cells_system_accuracy_ok','min_system_accuracy','power_W','gap_stage2','solve_s'});
writetable(T, fullfile(outDir, 'summary_runs.csv'));
save(fullfile(outDir, 'allocs.mat'), 'allocs', 'C', '-v7.3');
fprintf('\nSaved %s (%d runs, %.0f s)\n', fullfile(outDir, 'summary_runs.csv'), height(T), toc(t0));
end

function t = tern(c, a, b), if c, t = a; else, t = b; end, end
