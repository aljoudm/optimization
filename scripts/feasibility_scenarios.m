function R = feasibility_scenarios()
% FEASIBILITY_SCENARIOS  Before the main MILP: can the drone scenarios (3 / 6 / 12 drones) satisfy the per-cell requirements?
%
% With DAS in every cell (needed by the classifier requirement) a cell needs a drone within reach when
%   * detection with DAS alone is below AD0 (Pd_DAS < AD0, camera coverage is only a small part of a cell), and/or
%   * the cell is Hot (two independent layers at every point = DAS + drone).
% kreq(i) = fewest drones at the serving station for which SOME state of cell i meets every constraint (0 = no drone needed,
% Inf = impossible). A station with n drones serves every cell within reach whose kreq <= n. For each scenario this solves a
% small MILP: maximise the number of cells served with at most N drones (and the same for Hot cells only), and also finds the
% smallest number of drones that serves every cell.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
P0 = load_params();   cells = load_cells(P0);   A = place_existing_assets(P0, cells);
hot = cells.zoneCode == 3;   n = cells.n;   sc = P0.assets.scenarios;
R = struct('Pd_DAS', {}, 'Pc_DAS', {}, 'scenario', {}, 'N_drone', {}, 'cells_unserved', {}, 'hot_unserved', {}, 'min_drones_all', {});
fprintf('Drone availability: T_drone %g min, T_charge %g min -> A(1) = %.3f, K = %d drones are enough per station\n', P0.v.T_drone_min, P0.v.T_charge_min, P0.v.T_drone_min/(P0.v.T_drone_min+P0.v.T_charge_min), ceil(1/(P0.v.T_drone_min/(P0.v.T_drone_min+P0.v.T_charge_min))));
for pdd = [0.969 1.0]
    for pcd = [0.983 0.993]
        P = P0;   P.v.Pd_DAS = pdd;   P.v.Pc_DAS = pcd;
        M = coverage_model(P, cells, A);   K = M.K;
        kreq = inf(n, 1);
        for i = 1:n
            for k = 0:K, if M.ok(i, 2, k + 1), kreq(i) = k; break; end, end
        end
        need = isfinite(kreq) & kreq >= 1;
        fprintf('\n=== Pd_DAS %.3f, Pc_DAS %.3f (classifier definition) ===\n', pdd, pcd);
        fprintf('cells needing a drone: %d (Hot %d of %d); impossible cells: %d; with DAS alone enough: %d\n', nnz(need), nnz(need & hot), nnz(hot), nnz(~isfinite(kreq)), nnz(kreq == 0));
        fprintf('system-level Pc (extra result) of the best state per cell: min %.3f max %.3f; cells whose best state meets AC0 as a whole system: %d\n', min(M.bestPc), max(M.bestPc), nnz(any(reshape(M.okSystemAC0, n, []), 2)));
        for s = 1:numel(sc)
            [nServed, nHotServed] = deal(0);
            [servedAll, ~] = max_served(M.reach, kreq, need, K, sc(s).N_drone, ones(n, 1));
            [servedHot, ~] = max_served(M.reach, kreq, need, K, sc(s).N_drone, double(hot) * 1000 + 1);
            nNeed = nnz(need);   nHotNeed = nnz(need & hot);
            hotServed = nnz(servedHot & hot);   allServed = nnz(servedAll);
            minN = min_drones(M.reach, kreq, need, K);
            fprintf('  scenario %-6s (%2d drones): cells served %3d / %3d needing a drone -> unserved %3d (Hot unserved, best case for Hot: %2d of %2d)   | fewest drones to serve all: %d\n', ...
                sc(s).name, sc(s).N_drone, allServed, nNeed, nNeed - allServed, nHotNeed - hotServed, nHotNeed, minN);
            R(end+1) = struct('Pd_DAS', pdd, 'Pc_DAS', pcd, 'scenario', sc(s).name, 'N_drone', sc(s).N_drone, ...
                'cells_unserved', nNeed - allServed, 'hot_unserved', nHotNeed - hotServed, 'min_drones_all', minN); %#ok<AGROW>
        end
    end
end
end

function [served, u] = max_served(reach, kreq, need, K, N, w)
% maximise sum_i w_i y_i : y_i = 1 if some station s with n_s >= kreq(i) reaches cell i; sum_s n_s <= N
[nS, n] = size(reach);   cellsNeed = find(need);   m = numel(cellsNeed);
nu = nS * K;   nv = nu + m;                                  % u(s,k) binaries then y
col = @(s, k) (s - 1) * K + k;
f = zeros(nv, 1);   f(nu + 1:end) = -w(cellsNeed);
Aeq = []; beq = [];
rowsA = {};   b = [];
% one drone level per station
Aone = sparse(nS, nv);   for s = 1:nS, Aone(s, col(s, 1):col(s, K)) = 1; end
% drones budget
Abud = sparse(1, nv);   for s = 1:nS, for k = 1:K, Abud(1, col(s, k)) = k; end, end
% cover: y_i - sum_{s reaches i, k >= kreq(i)} u(s,k) <= 0
Acov = sparse(m, nv);
for q = 1:m
    i = cellsNeed(q);   Acov(q, nu + q) = 1;
    for s = find(reach(:, i))', for k = kreq(i):K, Acov(q, col(s, k)) = -1; end, end
end
Aineq = [Aone; Abud; Acov];   bineq = [ones(nS, 1); N; zeros(m, 1)];
opts = optimoptions('intlinprog', 'Display', 'off', 'MaxTime', 120);
x = intlinprog(f, 1:nv, Aineq, bineq, Aeq, beq, zeros(nv, 1), ones(nv, 1), opts);
served = false(n, 1);   served(cellsNeed(round(x(nu + 1:end)) == 1)) = true;   u = round(x(1:nu));
end

function minN = min_drones(reach, kreq, need, K)
% fewest drones such that every cell needing one is served (inf if impossible)
[nS, n] = size(reach);   cellsNeed = find(need);   m = numel(cellsNeed);
nu = nS * K;   col = @(s, k) (s - 1) * K + k;
f = zeros(nu, 1);   for s = 1:nS, for k = 1:K, f(col(s, k)) = k; end, end
Aone = sparse(nS, nu);   for s = 1:nS, Aone(s, col(s, 1):col(s, K)) = 1; end
Acov = sparse(m, nu);
for q = 1:m
    i = cellsNeed(q);
    for s = find(reach(:, i))', for k = kreq(i):K, Acov(q, col(s, k)) = -1; end, end
end
opts = optimoptions('intlinprog', 'Display', 'off', 'MaxTime', 120);
[~, fval, flag] = intlinprog(f, 1:nu, [Aone; Acov], [ones(nS, 1); -ones(m, 1)], [], [], zeros(nu, 1), ones(nu, 1), opts);
if flag > 0, minN = round(fval); else, minN = inf; end
end
