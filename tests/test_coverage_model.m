function test_coverage_model()
% TEST_COVERAGE_MODEL  fuse_classification and coverage_model against closed-form numbers written by hand.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
tol = 1e-12;

%% ---- fuse_classification: hand-computed cases ------------------------------------------------------------------
assert(abs(fuse_classification([0.9 0.85 0.85]) - (0.9*0.85*0.85 + 0.9*0.85*0.15 + 0.9*0.15*0.85 + 0.1*0.85*0.85)) < tol, '3 sensors = P(at least 2 right)');
assert(abs(fuse_classification([0.9 0.85]) - 0.9) < tol, '2 sensors: the more accurate one decides');
assert(abs(fuse_classification([0.8 0.8]) - 0.8) < tol, 'tie counts 1/2');
assert(abs(fuse_classification([0.9 0.85], 'independent_or') - 0.985) < tol, 'or bound');
assert(abs(fuse_classification([0.9 0.85], 'max') - 0.9) < tol, 'max');
assert(abs(fuse_classification(0.9) - 0.9) < tol, 'single sensor');
% a very accurate sensor outvotes two weaker ones: 0.99 vs 0.85+0.85 -> ln(99)=4.60 > 2*1.73
assert(abs(fuse_classification([0.99 0.85 0.85]) - 0.99) < 1e-9, 'dominant sensor');
rng(1);
for t = 1:200                                           % properties: max <= weighted vote <= or-bound
    p = 0.55 + 0.44 * rand(1, randi(4));
    wv = fuse_classification(p);
    assert(wv >= max(p) - 1e-12 && wv <= fuse_classification(p, 'independent_or') + 1e-12, 'ordering of the fusion rules');
end

%% ---- coverage_model ---------------------------------------------------------------------------------------------
P = load_params();   v = P.v;   cells = load_cells(P);   A = place_existing_assets(P, cells);   M = coverage_model(P, cells, A);
n = cells.n;   K = M.K;
assert(K == 3 && abs(M.A1 - 47/107) < tol && abs(M.A(3) - 2*47/107) < tol && M.A(4) == 1, 'drone availability');
iNo  = find(A.coverFrac == 0, 1);                      % a cell without camera
iCam = find(A.coverFrac > 0 & A.coverFrac < 1, 1);     % a cell with partial camera coverage
f = A.coverFrac(iCam);
pD = v.Pd_DAS;  cD = v.Pc_DAS;  pC = v.Pd_CCTV;  cC = v.Pc_CCTV;  pR = v.Pd_drone;  cR = v.Pc_drone;

% (1) no camera, state (DAS only)
assert(abs(M.Pd(iNo, 2, 1) - pD) < tol && abs(M.Pc(iNo, 2, 1) - cD) < tol, 'DAS only');
% (2) no camera, no DAS, no drone: nothing detects
assert(M.Pd(iNo, 1, 1) == 0 && M.Pc(iNo, 1, 1) == 0, 'empty state');
% (3) no camera, drone only with 2 drones
d2 = pR * M.A(3);
assert(abs(M.Pd(iNo, 1, 3) - d2) < tol && abs(M.Pc(iNo, 1, 3) - cR) < tol, 'drone only');
% (4) no camera, DAS + 2 drones: hand formula (DAS more accurate than drone, so it decides when both detect)
det = 1 - (1 - pD) * (1 - d2);
cor = pD*(1-d2)*cD + (1-pD)*d2*cR + pD*d2*max(cD, cR);
assert(abs(M.Pd(iNo, 2, 3) - det) < tol && abs(M.Pc(iNo, 2, 3) - cor/det) < tol, 'DAS + drones, hand formula');
% (5) partial camera coverage, DAS only: uncovered part f0 and covered part f
det0 = pD;  cor0 = pD*cD;
det1 = 1 - (1-pD)*(1-pC);  cor1 = pD*(1-pC)*cD + (1-pD)*pC*cC + pD*pC*max(cD, cC);
Pd5 = (1-f)*det0 + f*det1;  Pc5 = ((1-f)*cor0 + f*cor1) / Pd5;
assert(abs(M.Pd(iCam, 2, 1) - Pd5) < tol && abs(M.Pc(iCam, 2, 1) - Pc5) < tol, 'partial coverage, hand formula');
% (6) camera only (no DAS, no drone): detection = f * Pd_CCTV, classification = Pc_CCTV
assert(abs(M.Pd(iCam, 1, 1) - f*pC) < tol && abs(M.Pc(iCam, 1, 1) - cC) < tol, 'camera only');
% (7) detection never decreases when a layer is added; classification bounds are ordered
assert(all(M.Pd(:, 2, :) >= M.Pd(:, 1, :) - tol, 'all') && all(diff(M.Pd, 1, 3) >= -tol, 'all'), 'Pd monotone in layers');
assert(all(M.Pc_or(:) >= M.Pc(:) - 1e-9) && all(M.Pc(:) >= M.Pc_max(:) - 1e-9), 'Pc_or >= Pc >= Pc_max');
% (8) layers: Hot cells need 2 -> DAS + a drone; no state with one layer satisfies a Hot cell
hot = find(cells.zoneCode == 3);
assert(all(M.need(hot) == 2) && all(M.need(cells.zoneCode ~= 3) == 1), 'layer requirement by zone');
assert(all(M.layers(hot, 2, 2) == 2) && all(M.layers(hot, 2, 1) == 1), 'layers DAS + drone');
assert(~any(M.ok(hot, 2, 1), 'all') && ~any(M.okDet(hot, 2, 1), 'all'), 'Hot cell with DAS only must fail the 2-layer rule');
% (9) drone reach
assert(size(M.reach, 2) == n && all(any(M.reach, 1)), 'every cell is reachable from at least one candidate station');
assert(max(sum(M.reach, 2)) >= 40, 'one station should cover about 40 cells');

%% ---- feasibility report with the current PLACEHOLDERS ----------------------------------------------------------------
fprintf('Placeholders: Pd_DAS %.2f, Pc_DAS %.2f, Pc_drone %.2f, T_charge %g min -> availability A(1..3) = %.3f %.3f %.3f\n', v.Pd_DAS, v.Pc_DAS, v.Pc_drone, v.T_charge_min, M.A(2:4));
fprintf('Cells for which SOME state meets detection + layers + classification (AD0 = AC0 = %.2f): %d of %d\n', v.AD0, nnz(M.cellOk), n);
fprintf('Cells for which some state meets detection + layers only (classification reported as a gap)  : %d of %d\n', nnz(M.cellOkDet), n);
fprintf('Best Pc any state reaches (%s): min %.3f, max %.3f; with the optimistic 1-prod(1-p) bound: max %.3f\n', M.fusion, min(M.bestPc), max(M.bestPc), max(M.Pc_or, [], 'all'));
% sensitivity: what do Pd_DAS and Pc_DAS have to be for classification to become satisfiable?
fprintf('Cells that can meet detection + layers + classification, by (Pd_DAS, Pc_DAS):\n');
for pdd = [0.95 0.98 0.99]
    for pcd = [0.98 0.99 0.995]
        P2 = P;  P2.v.Pd_DAS = pdd;  P2.v.Pc_DAS = pcd;   M2 = coverage_model(P2, cells, A);
        fprintf('  Pd_DAS %.3f, Pc_DAS %.3f : %3d of %d (Hot: %2d of %d)\n', pdd, pcd, nnz(M2.cellOk), n, nnz(M2.cellOk(hot)), numel(hot));
        if pdd == 0.99 && pcd == 0.995, assert(nnz(M2.cellOk) == n, 'with a very good DAS every cell must be satisfiable'); end
    end
end
fprintf('test_coverage_model: all checks passed.\n');
end
