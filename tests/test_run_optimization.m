function test_run_optimization()
% TEST_RUN_OPTIMIZATION  The MILP solution is re-checked by an independent routine and against hand-derived bounds.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
P = load_params();   v = P.v;   cells = load_cells(P);   A = place_existing_assets(P, cells);   M = coverage_model(P, cells, A);
sc = P.assets.scenarios;
fprintf('Optimization Toolbox available: %d\n', license('test', 'Optimization_Toolbox'));

%% medium scenario, base values
al = run_optimization(cells, P, 'Scenario', sc(2), 'Assets', A, 'Model', M, 'Verbose', false);
verify(al, cells, P, M, A);
% hand bounds: every cell needs DAS (classifier requirement), so the fibre covers the whole ring and >= ceil(L / (2 R_DAS)) sites are needed
L = cells.ring.L;   nMin = ceil(L / (v.N_ch_active * v.R_DAS_m));
assert(al.n_interrogators == nMin, 'interrogators %d vs hand bound %d', al.n_interrogators, nMin);
assert(abs(al.fibre_km - L / 1e3) < 1e-6, 'fibre must cover the whole ring');
assert(abs(al.C_total - (nMin * v.C_DAS_int + v.C_fiber_per_km * L / 1e3)) < 1e-3, 'C_total = %d interrogators + fibre of the whole ring', nMin);
assert(al.n_violations == 0 && al.feasible, 'medium scenario must be feasible');
assert(al.drones.total <= sc(2).N_drone, 'drone budget');
fprintf('medium: C_total %.2f = %d x %.0f + %.2f km x %.0f ; drones %d\n', al.C_total, nMin, v.C_DAS_int, L/1e3, v.C_fiber_per_km, al.drones.total);

%% low scenario: a solution WITH a gap, 3 cells below AD0, and 4 drones would remove it
al3 = run_optimization(cells, P, 'Scenario', sc(1), 'Assets', A, 'Model', M, 'Verbose', false);
verify(al3, cells, P, M, A);
assert(al3.n_violations == 3 && al3.min_drones_no_gap == 4 && al3.drones.total <= 3, 'low scenario: 3 cells with a gap, 4 drones needed');
assert(all(al3.cells.Pd(al3.violations) < v.AD0) && all(al3.cells.Pd(al3.violations) > 0.96), 'violating cells keep DAS (Pd just below AD0)');
fprintf('low: %d cells below AD0 (Pd %s), fewest drones without gap %d\n', al3.n_violations, mat2str(al3.cells.Pd(al3.violations)', 4), al3.min_drones_no_gap);

%% Pd_DAS = 1.0: DAS alone detects; drones are only needed for the 2-layer rule of the Hot cells
P1 = P;   P1.v.Pd_DAS = 1.0;   M1 = coverage_model(P1, cells, A);
al1 = run_optimization(cells, P1, 'Scenario', sc(1), 'Assets', A, 'Model', M1, 'Verbose', false);
verify(al1, cells, P1, M1, A);
assert(al1.n_violations == 0 && al1.drones.total <= 3, 'Pd_DAS = 1: low scenario is feasible with <= 3 drones');
assert(all(al1.cells.drones_serving(cells.zoneCode == 3) >= 1), 'every Hot cell needs a drone layer');
assert(abs(al1.C_total - al.C_total) < 1e-3, 'cost does not depend on the drones');
fprintf('Pd_DAS = 1.0: low scenario feasible with %d drones at %d stations\n', al1.drones.total, al1.drones.stations);
fprintf('test_run_optimization: all checks passed.\n');
end

function verify(al, cells, P, M, A)
% Independent re-check of a solution (does not use the MILP model): requirements per cell, fibre continuity and reach, power, drones.
v = P.v;   n = cells.n;   runs = cells.runs;   R = numel(runs.cell);   L = cells.ring.L;   pos = (runs.s0 + runs.s1) / 2;
T = al.cells;
% (1) each cell: Pd / layers / classification of the chosen state, recomputed from the model arrays
for i = 1:n
    x = T.DAS(i);   k = T.drones_serving(i);
    ok = M.Pd(i, x+1, k+1) >= v.AD0 - 1e-12 && M.layers(i, x+1, k+1) >= M.need(i) && x == 1 && v.Pc_DAS >= v.AC0 - 1e-12;
    assert(ok == ~T.violates(i), 'cell %d: requirement flag inconsistent', i);
    if k >= 1, assert(T.station(i) > 0 && al.drones.n(find(ismember(al.drones.station_xy, M.stationXY(T.station(i), :), 'rows'), 1)) >= k, 'cell %d: no station with %d drones', i, k); end
end
% (2) the drone station that serves a cell is within reach and holds enough drones; budget
assert(al.drones.total <= al.scenario.N_drone, 'drone budget exceeded');
for i = find(T.drones_serving > 0)'
    assert(M.reach(T.station(i), i), 'cell %d is outside the reach of its station', i);
end
% (3) fibre: DAS cells have fibre on all their runs; every fibre run is reachable from a site with fibre all the way
z = al.fibre_runs;   y = false(R, 1);   y(al.interrogators.run) = true;
for i = find(T.DAS)', assert(all(z(cells.cellRuns{i})), 'cell %d: DAS without fibre on all runs', i); end
Rmax = v.R_DAS_m - v.L_leadin_m;   served = y;
for k = find(y)'
    j = k;                                                      % right channel
    for t = 1:R - 1, j = mod(j, R) + 1;     if mod(runs.s1(j) - pos(k), L) > Rmax + 1e-9 || ~z(j), break; end, served(j) = true; end
    j = k;                                                      % left channel
    for t = 1:R - 1, j = mod(j - 2, R) + 1; if mod(pos(k) - runs.s0(j), L) > Rmax + 1e-9 || ~z(j), break; end, served(j) = true; end
end
assert(all(served(z)), 'some fibre run is not served by any interrogator within R_DAS through continuous fibre');
% (4) power, cost
assert((v.P_DAS + v.P_shelter) * nnz(y) < v.P0, 'power constraint');
assert(abs(al.C_total - (nnz(y) * (v.C_DAS_int + v.C_shelter) + v.C_fiber_per_km * sum(runs.len(z)) / 1e3)) < 1e-6, 'cost');
end
