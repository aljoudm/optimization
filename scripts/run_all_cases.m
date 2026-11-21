function T = run_all_cases()
% RUN_ALL_CASES  Stage 4 driver: the MILP for every scenario and sensitivity case.
%   scenarios (drones 3 / 6 / 12 and patrols 5 / 15 / 30)  x  interdiction cases (real crossings only / completed with filler)
%   x  Pd_DAS (0.969 base, 0.98 = AD0, 1.0)  x  Pc_DAS (0.983 / 0.993, the two ends of wang2024)
%   + C_fiber x0.5 and x2 on the base case.
% Results: results/summary_runs.csv and results/allocs.mat (every alloc struct).
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
outDir = fullfile(here, '..', 'results');   if ~exist(outDir, 'dir'), mkdir(outDir); end
P0 = load_params();   cells = load_cells(P0);
fprintf('Optimization Toolbox: %s\n', tern(license('test', 'Optimization_Toolbox') && exist('intlinprog', 'file') > 0, 'available', 'MISSING'));
interd = struct('name', {'real22', 'with_filler32'}, 'allow_filler', {false, true});
scen = P0.assets.scenarios;   pdList = [0.969 0.98 1.0];   pcList = [0.983 0.993];
rows = {};   allocs = {};   t0 = tic;   reachCache = [];
caseList = struct('inter', {}, 'sc', {}, 'pd', {}, 'pc', {}, 'cfib', {});
for a = 1:numel(interd), for s = 1:numel(scen), for pd = pdList, for pc = pcList
    caseList(end+1) = struct('inter', a, 'sc', s, 'pd', pd, 'pc', pc, 'cfib', 1); %#ok<AGROW>
end, end, end, end
for f = [0.5 2], caseList(end+1) = struct('inter', 2, 'sc', 2, 'pd', 0.969, 'pc', 0.983, 'cfib', f); end     % C_fiber sensitivity (base case)
lastInter = 0;
for c = 1:numel(caseList)
    cs = caseList(c);   P = P0;
    P.assets.allow_filler = interd(cs.inter).allow_filler;   P.v.Pd_DAS = cs.pd;   P.v.Pc_DAS = cs.pc;   P.v.C_fiber_per_km = P0.v.C_fiber_per_km * cs.cfib;
    if cs.inter ~= lastInter, A = place_existing_assets(P, cells);   lastInter = cs.inter; end
    M = coverage_model(P, cells, A, reachCache);   reachCache = struct('reach', M.reach, 'stationXY', M.stationXY);
    fprintf('\n--- case %d/%d: %s | scenario %s (%d drones) | Pd_DAS %.3f | Pc_DAS %.3f | C_fiber x%g | elapsed %.0f s\n', c, numel(caseList), interd(cs.inter).name, scen(cs.sc).name, scen(cs.sc).N_drone, cs.pd, cs.pc, cs.cfib, toc(t0));
    al = run_optimization(cells, P, 'Scenario', scen(cs.sc), 'Assets', A, 'Model', M, 'Verbose', true);
    al.caseInfo = struct('interdiction', interd(cs.inter).name, 'Pd_DAS', cs.pd, 'Pc_DAS', cs.pc, 'C_fiber_factor', cs.cfib);
    allocs{end+1} = al; %#ok<AGROW>
    nSites = nnz([A.sites.in]);
    rows(end+1, :) = {string(interd(cs.inter).name), nSites, string(scen(cs.sc).name), scen(cs.sc).N_drone, scen(cs.sc).N_patrol, cs.pd, cs.pc, cs.cfib, ...
        al.C_total, al.n_interrogators, al.fibre_km, nnz(al.cells.DAS), al.drones.total, al.drones.stations, al.n_violations, al.min_drones_no_gap, ...
        al.system_accuracy_ok, al.power.total_W, al.power.within_P0, al.elapsed_s}; %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'interdiction_case','camera_sites','scenario','N_drone','N_patrol','Pd_DAS','Pc_DAS','C_fiber_factor', ...
    'C_total_SAR','interrogators','fibre_km','DAS_cells','drones_used','drone_stations','cells_below_requirement','fewest_drones_no_gap', ...
    'cells_system_accuracy_ok','power_W','within_P0','solve_s'});
writetable(T, fullfile(outDir, 'summary_runs.csv'));
save(fullfile(outDir, 'allocs.mat'), 'allocs', 'caseList', '-v7.3');
fprintf('\nSaved %s (%d runs, %.0f s)\n', fullfile(outDir, 'summary_runs.csv'), height(T), toc(t0));
end

function t = tern(c, a, b), if c, t = a; else, t = b; end, end
