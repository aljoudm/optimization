function alloc = run_base_case(timeLimit, Pd_DAS)
% RUN_BASE_CASE  Base case: medium scenario (6 drones), 22 real interdiction crossings, Pd_DAS = 0.969 (or the given value), Pc_DAS = 0.983.
% timeLimit (s) = total time per case (default 1800 = 30 min); progress is printed every 10 minutes.
if nargin < 1, timeLimit = 1800; end
if nargin < 2, Pd_DAS = 0.969; end
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
P = load_params();   P.assets.allow_filler = false;   P.v.Pd_DAS = Pd_DAS;   P.v.Pc_DAS = 0.983;
cells = load_cells(P);   A = place_existing_assets(P, cells);   M = coverage_model(P, cells, A);
sc = P.assets.scenarios(2);
fprintf('Base case: scenario %s (%d drones), %d camera sites, Pd_DAS %.3f, Pc_DAS %.3f, R_resp %.0f m, time limit %d s\n', sc.name, sc.N_drone, nnz([A.sites.in]), P.v.Pd_DAS, P.v.Pc_DAS, P.v.R_resp_m, timeLimit);
fprintf('Detection without DAS: best Pd of any cell %.4f (cameras cover %.2f%% of the boundary); with DAS only: Pd %.4f (AD0 %.2f)\n', max(M.bestPd0), 100 * sum(A.coveredLen_m) / cells.ring.L, P.v.Pd_DAS, P.v.AD0);
alloc = run_optimization(cells, P, 'Scenario', sc, 'Assets', A, 'Model', M, 'ProgressSeconds', 600, 'TimeLimit', timeLimit);
if ~exist(fullfile(here, '..', 'results'), 'dir'), mkdir(fullfile(here, '..', 'results')); end
save(fullfile(here, '..', 'results', 'base_case.mat'), 'alloc', 'cells');
s2 = alloc.solver.stage2;
fprintf('Stage 2: %s (gap %.3g%%)\n', s2.message(1:min(end, 100)), 100 * s2.relativegap);
T = alloc.cells;   zn = {'Cold', 'Neutral', 'Hot'};
for k = [3 2 1]
    in = cells.zoneCode == k;
    fprintf('%-8s cells %2d | with DAS %2d | without DAS %2d | verified by drone %2d | Pd min %.4f max %.4f\n', zn{k}, nnz(in), nnz(T.DAS(in)), nnz(~T.DAS(in)), nnz(T.verified(in)), min(T.Pd(in)), max(T.Pd(in)));
end
fprintf('cells below a requirement: %d (Hot unverified: %d)\n', alloc.n_violations, nnz(cells.zoneCode == 3) - alloc.hot_verified);
fprintf('system accuracy (extra result): cells with Pc >= AC0: %d of %d; min %.4f max %.4f\n', alloc.system_accuracy_ok, height(T), min(T.Pc_system), max(T.Pc_system));
ring = cells.ring;
for s = 1:size(alloc.drones.station_xy, 1)
    fprintf('drone station %d: lat %.4f lon %.4f, %d drone(s)\n', s, ring.lat0 + alloc.drones.station_xy(s, 2) / ring.ky, ring.lon0 + alloc.drones.station_xy(s, 1) / ring.kx, alloc.drones.n(s));
end
fprintf('fibre %.1f km of %.1f km; fibre share of C_total %.1f%%\n', alloc.fibre_km, ring.L / 1e3, 100 * alloc.fibre_share);
end
