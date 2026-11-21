function test_coverage_model()
% TEST_COVERAGE_MODEL  fuse_classification and coverage_model (drones = verification layer) against closed-form numbers written by hand.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
tol = 1e-12;

%% ---- fuse_classification: hand-computed cases ------------------------------------------------------------------
assert(abs(fuse_classification([0.9 0.85 0.85]) - (0.9*0.85*0.85 + 0.9*0.85*0.15 + 0.9*0.15*0.85 + 0.1*0.85*0.85)) < tol, '3 sensors = P(at least 2 right)');
assert(abs(fuse_classification([0.9 0.85]) - 0.9) < tol, '2 sensors: the more accurate one decides');
assert(abs(fuse_classification([0.8 0.8]) - 0.8) < tol, 'tie counts 1/2');
assert(abs(fuse_classification([0.9 0.85], 'independent_or') - 0.985) < tol, 'or bound');
assert(abs(fuse_classification([0.9 0.85], 'max') - 0.9) < tol, 'max');
assert(abs(fuse_classification(0.9) - 0.9) < tol, 'single sensor');
assert(abs(fuse_classification([0.99 0.85 0.85]) - 0.99) < 1e-9, 'dominant sensor');
rng(1);
for t = 1:200
    p = 0.55 + 0.44 * rand(1, randi(4));
    wv = fuse_classification(p);
    assert(wv >= max(p) - 1e-12 && wv <= fuse_classification(p, 'independent_or') + 1e-12, 'ordering of the fusion rules');
end

%% ---- coverage_model ---------------------------------------------------------------------------------------------
P = load_params();   v = P.v;   cells = load_cells(P);   A = place_existing_assets(P, cells);   M = coverage_model(P, cells, A);
n = cells.n;   st = M.states;
assert(M.K == 2 && abs(M.A1 - 47/74) < tol && M.A(3) == 1, 'drone availability (T_charge 27 min)');
assert(v.R_resp_m == min(11000, 15 * (300 - 10)) && M.R_resp == 4350, 'R_resp = min(11 km, 15 m/s x (300 - 10) s) = 4350 m');
iNo  = find(A.coverFrac == 0, 1);                      % a cell without camera
iCam = find(A.coverFrac > 0 & A.coverFrac < 1, 1);     % a cell with partial camera coverage
f = A.coverFrac(iCam);
pD = v.Pd_DAS;  cD = v.Pc_DAS;  pC = v.Pd_CCTV;  cC = v.Pc_CCTV;  cR = v.Pc_drone;
s00 = M.idx(0, [0 0]);   s10 = M.idx(1, [0 0]);   s1a = M.idx(1, [0 1]);   s1b = M.idx(1, [1 0]);   s1c = M.idx(1, [2 0]);

% (1) no camera: DAS only, nothing else
assert(abs(M.Pd(iNo, s10) - pD) < tol && abs(M.Pc(iNo, s10) - cD) < tol, 'DAS only');
assert(M.Pd(iNo, s00) == 0 && M.Pc(iNo, s00) == 0, 'empty state');
% (2) drones do NOT detect: Pd does not depend on the drone units, only on x and the camera
assert(all(abs(M.Pd(:, st.x == 1) - M.Pd(:, s10)) < tol, 'all') && all(abs(M.Pd(:, st.x == 0) - M.Pd(:, s00)) < tol, 'all'), 'Pd must not depend on drones');
% (3) no camera, DAS + a verifying station with 2 drones (ready prob 1): the drone adds a vote, two voters -> the better one decides
assert(abs(M.Pc(iNo, s1a) - fuse_classification([cD cR])) < tol && abs(M.Pc(iNo, s1a) - cD) < tol, 'verification vote');
% (4) partial camera coverage, DAS only: hand formula
det0 = pD;  cor0 = pD*cD;
det1 = 1 - (1-pD)*(1-pC);  cor1 = pD*(1-pC)*cD + (1-pD)*pC*cC + pD*pC*max(cD, cC);
Pd4 = (1-f)*det0 + f*det1;  Pc4 = ((1-f)*cor0 + f*cor1) / Pd4;
assert(abs(M.Pd(iCam, s10) - Pd4) < tol && abs(M.Pc(iCam, s10) - Pc4) < tol, 'partial coverage, hand formula');
% (5) camera only (no DAS): detection = f * Pd_CCTV
assert(abs(M.Pd(iCam, s00) - f*pC) < tol && abs(M.Pc(iCam, s00) - cC) < tol, 'camera only');
% (6) ready probability and the verification layer: 1 drone 0.635, 2 drones 1.0, two stations with 1 drone 1-(1-0.635)^2 = 0.867
assert(abs(st.a_ver(s1b) - 47/74) < tol && st.a_ver(s1a) == 1 && abs(st.a_ver(s1c) - (1 - (1 - 47/74)^2)) < tol, 'a_ver');
assert(~st.ver(s1b) && st.ver(s1a) && ~st.ver(s1c), 'A_ver_min = 0.9: only the 2-drone station is enough');
hot = find(cells.zoneCode == 3);
assert(all(M.need(hot) == 2) && all(M.need(cells.zoneCode ~= 3) == 1), 'layer requirement by zone');
assert(all(M.layers(hot, s1a) == 2) && all(M.layers(hot, s10) == 1) && all(M.layers(hot, s1b) == 1), 'layers = DAS + verification');
% (7) AD0: with Pd_DAS = 0.969 no state meets detection (camera covers 2.6%); the gap is small
assert(~any(M.okDet, 'all'), 'Pd_DAS 0.969 < AD0 everywhere');
assert(max(M.Pd(:, s10)) < v.AD0 && min(M.Pd(:, s10)) >= pD - tol && max(v.AD0 - M.Pd(:, s10)) < 0.0115, 'detection gap <= 0.011');
P1 = P;   P1.v.Pd_DAS = 0.98;   M1 = coverage_model(P1, cells, A, struct('reach', M.reach, 'stationXY', M.stationXY, 'radius', M.reach_radius));
assert(all(M1.okDet(setdiff(1:n, hot), s10)) && ~any(M1.okDet(hot, s10)) && all(M1.okDet(hot, s1a)), 'Pd_DAS 0.98: DAS alone meets AD0, Hot cells also need verification');
% (8) classifier check and the other definition
assert(M.classifier_ok, 'Pc_DAS 0.983 >= AC0');
P2 = P;   P2.opts.ac0_definition = 'system';   M2 = coverage_model(P2, cells, A, struct('reach', M.reach, 'stationXY', M.stationXY, 'radius', M.reach_radius));
assert(all(M2.Pc(:) == M.Pc(:)) && all(M2.ok(:) <= M.ok(:)), 'system definition only adds a per-cell condition');
% (9) bounds
assert(all(M.Pc_or(:) >= M.Pc(:) - 1e-9) && all(M.Pc(:) >= M.Pc_max(:) - 1e-9), 'Pc_or >= Pc >= Pc_max');
assert(all(any(M.reach, 1)), 'every cell is reachable from a candidate station');
fprintf('R_resp %.0f m; availability %s; best Pd without DAS %.4f; Pd_DAS 0.969 -> cells meeting detection: %d\n', M.R_resp, mat2str(M.A, 3), max(M.bestPd0), nnz(any(M.okDet, 2)));
fprintf('test_coverage_model: all checks passed.\n');
end
