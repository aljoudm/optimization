function test_run_optimization()
% TEST_RUN_OPTIMIZATION  The MILP solution is re-checked by an independent routine and against hand-derived bounds.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
P = load_params();   P.assets.allow_filler = false;   v = P.v;   cells = load_cells(P);   A = place_existing_assets(P, cells);   M = coverage_model(P, cells, A);
sc = P.assets.scenarios;   hot = cells.zoneCode == 3;   L = cells.ring.L;
fprintf('Optimization Toolbox available: %d\n', license('test', 'Optimization_Toolbox'));
nMin = ceil(L / (v.N_ch_active * v.R_DAS_m));   fullCost = nMin * (v.C_DAS_int + v.C_shelter) + v.C_fiber_per_km * L / 1e3;

%% base case: medium scenario, Pd_DAS 0.969 (below AD0: elastic detection, never infeasible)
al = run_optimization(cells, P, 'Scenario', sc(2), 'Assets', A, 'Model', M, 'Verbose', false);
verify(al, cells, P, M);
% hand bound: without DAS a cell detects with at most f*Pd_CCTV << AD0, so any allocation that minimises the detection shortfall puts DAS everywhere
assert(max(M.bestPd0) < 0.25, 'without DAS no cell comes near AD0');
assert(al.cells_with_DAS == cells.n && al.n_interrogators == nMin && abs(al.fibre_km - L/1e3) < 1e-6, 'DAS everywhere: %d interrogators, whole ring', nMin);
assert(abs(al.C_total - fullCost) < 1e-3, 'C_total = %d interrogators + fibre of the whole ring', nMin);
assert(al.detection_gap.cells_below_AD0 == cells.n && al.detection_gap.max < 0.0115 && al.detection_gap.max > 0.0105, 'elastic detection gap about 0.011');
assert(al.drones.total <= sc(2).N_drone && al.hot_verified == 22, 'medium: 6 drones verify 22 of the 36 Hot cells');
assert(abs(al.A_ver_hot.sum - 22) < 1e-9, 'stage 3: sum of A_ver over Hot cells');
assert(al.min_drones_no_gap == 14, 'fewest drones that verify every Hot cell (T_resp 300 s)');
fprintf('medium: C_total %.2f = full DAS; Hot verified %d/36; fewest drones for all Hot %d; detection gap max %.4f\n', al.C_total, al.hot_verified, al.min_drones_no_gap, al.detection_gap.max);

%% Pd_DAS = 0.98 (= AD0): detection is met by DAS alone, the remaining gap is verification of Hot cells
P1 = P;   P1.v.Pd_DAS = 0.98;   M1 = coverage_model(P1, cells, A, struct('reach', M.reach, 'stationXY', M.stationXY, 'radius', M.reach_radius));
al1 = run_optimization(cells, P1, 'Scenario', sc(1), 'Assets', A, 'Model', M1, 'Verbose', false);
verify(al1, cells, P1, M1);
assert(al1.detection_gap.cells_below_AD0 == 0 && al1.n_violations == nnz(hot) - al1.hot_verified && al1.hot_verified == 9, 'low scenario, Pd_DAS 0.98: 9 Hot cells verified, the rest is a verification gap');
assert(abs(al1.C_total - fullCost) < 1e-3, 'cost unchanged');
%% high scenario with 12 drones
al2 = run_optimization(cells, P1, 'Scenario', sc(3), 'Assets', A, 'Model', M1, 'Verbose', false);
verify(al2, cells, P1, M1);
assert(al2.hot_verified == 34 && al2.drones.total <= 12, 'high: 34 of 36 Hot cells verified');
% fewer cameras / the case with filler sites must not change the allocation
Pf = P1;   Pf.assets.allow_filler = true;   Af = place_existing_assets(Pf, cells);   Mf = coverage_model(Pf, cells, Af, struct('reach', M.reach, 'stationXY', M.stationXY, 'radius', M.reach_radius));
al3 = run_optimization(cells, Pf, 'Scenario', sc(2), 'Assets', Af, 'Model', Mf, 'Verbose', false);
al4 = run_optimization(cells, P1, 'Scenario', sc(2), 'Assets', A, 'Model', M1, 'Verbose', false);
assert(abs(al3.C_total - al4.C_total) < 1e-6 && al3.hot_verified == al4.hot_verified, '22 real vs 32 interdiction sites: same result');
fprintf('low (Pd 0.98): Hot verified %d/36; high: %d/36; 22 vs 32 sites identical\n', al1.hot_verified, al2.hot_verified);
fprintf('test_run_optimization: all checks passed.\n');
end

function verify(al, cells, P, M)
% Independent re-check of a solution (does not use the MILP model): fibre continuity and reach, cell states, drones, power, cost.
v = P.v;   n = cells.n;   runs = cells.runs;   R = numel(runs.cell);   L = cells.ring.L;   pos = (runs.s0 + runs.s1) / 2;   T = al.cells;
st = M.states;
for i = 1:n                                          % each cell is in a state of the model and the flags agree
    s = al.cell_state(i);
    assert(st.x(s) == T.DAS(i) && abs(M.Pd(i, s) - T.Pd(i)) < 1e-12, 'cell %d state', i);
    assert(T.verified(i) == st.ver(s), 'cell %d verification flag', i);
    assert(T.violates(i) == ~M.ok(i, s), 'cell %d violation flag', i);
end
assert(al.drones.total <= al.scenario.N_drone, 'drone budget exceeded');
assert(all(al.drones.n >= 1 & al.drones.n <= M.K), 'drones per station');
% a verified cell must have enough stations in reach holding drones: count ready-capacity per cell from the station list
idx = al.drones.station_index;   levels = al.drones.n;
for i = find(T.verified)'
    inReach = M.reach(idx, i);   assert(any(inReach), 'verified cell %d has no station within reach', i);
    a = 1 - prod(1 - M.A(levels(inReach) + 1));   assert(a >= v.A_ver_min - 1e-9, 'cell %d: ready probability %.3f below A_ver_min', i, a);
end
z = al.fibre_runs;   y = false(R, 1);   y(al.interrogators.run) = true;
for i = find(T.DAS)', assert(all(z(cells.cellRuns{i})), 'cell %d: DAS without fibre on all runs', i); end
Rmax = v.R_DAS_m - v.L_leadin_m;   served = y;
for k = find(y)'
    j = k;   for t = 1:R - 1, j = mod(j, R) + 1;     if mod(runs.s1(j) - pos(k), L) > Rmax + 1e-9 || ~z(j), break; end, served(j) = true; end
    j = k;   for t = 1:R - 1, j = mod(j - 2, R) + 1; if mod(pos(k) - runs.s0(j), L) > Rmax + 1e-9 || ~z(j), break; end, served(j) = true; end
end
assert(all(served(z)), 'some fibre run is not served by any interrogator within R_DAS through continuous fibre');
assert((v.P_DAS + v.P_shelter) * nnz(y) < v.P0, 'power constraint');
assert(abs(al.C_total - (nnz(y) * (v.C_DAS_int + v.C_shelter) + v.C_fiber_per_km * sum(runs.len(z)) / 1e3)) < 1e-6, 'cost');
end
