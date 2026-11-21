function M = coverage_model(P, cells, assets)
% COVERAGE_MODEL  Detection / classification / layers of every cell for every possible allocation state, plus drone reach.
%
%   M = coverage_model(P, cells, assets)
%
% A cell is described by a STATE (x, k):  x = 1 if DAS fibre is installed in the cell, k = number of drones at the station that
% serves the cell (0..K). Everything else (existing cameras, partial coverage) is fixed input. For every cell and state this
% function precomputes constants, so the MILP stays linear and exact (no log-linearisation).
%
% MODEL (all assumptions are stated; independence between sensors is assumed everywhere)
%  * Intruder position: uniform along the cell's boundary. A piece of boundary is either covered by an existing camera
%    (fraction f_i = assets.coverFrac(i)) or not (1 - f_i). DAS covers the whole cell when x = 1; a drone station covers the
%    whole cell when the cell is within reach (the reach test is done by the optimizer, drone_reach below).
%  * Drone availability (time): one drone flies T_drone minutes and recharges T_charge minutes, so it is available a fraction
%    A1 = T/(T + T_charge) of the time; k drones at the station give A(k) = min(1, k*A1). Its effective detection probability is
%    Pd_drone * A(k). K = ceil(1/A1) drones are the most that can still help.
%  * DETECTION at a point: sensor j detects with probability d_j; the point is detected if ANY sensor detects:
%    1 - prod(1 - d_j) (valid for detection). The cell value is the length-weighted average over its pieces.
%  * CLASSIFICATION (modelling point 2): only a sensor that detected can classify. For every subset S of the active sensors that
%    detected (probability prod over S of d_j times prod over the rest of (1 - d_j)), the fused accuracy is
%    fuse_classification(pc_S) (weighted vote). The cell value is the accuracy CONDITIONAL on detection:
%    Pc_i = E[correct and detected] / E[detected], both averaged over the pieces.
%  * LAYERS (Hot cells need >= 2 independent layers at EVERY point): layers = x + [k >= 1] + [f_i = 1]; since f_i < 1 the camera
%    layer never covers the whole cell, so in practice DAS + a drone are needed (strict, point-wise rule).
%
% Output struct (arrays indexed (cell, x+1, k+1))
%   Pd, PdWorst (worst point of the cell), Pc, Pc_or (optimistic bound), Pc_max (no-fusion bound), layers, need, ok
%   A (availability per k = 0..K), K, fusion (mode used), cellOk
%   bestPc(i)   best Pc any state can reach under the chosen fusion;  gapAC0(i) = max(0, AC0 - bestPc)
%   reach       logical (station, cell): the cell lies fully within Cov_drone of the candidate station
%   stationXY   candidate drone stations [m, local frame]

v = P.v;   o = P.opts;   ring = cells.ring;   n = cells.n;
if v.use_PS && any(isnan([v.Pd_PS_value v.Pc_PS_value v.Cov_PS_value])), error('coverage_model:PS', 'Point-Sensor values are TBD: set use_PS = false or fill them in load_params.'); end
assert(~o.drone_time_share, 'coverage_model:timeshare', 'drone_time_share is a sensitivity option that is not implemented yet.');

A1 = v.T_drone_min / (v.T_drone_min + v.T_charge_min);
K  = ceil(1 / A1 - 1e-12);
Ak = min(1, (0:K) * A1);
f  = assets.coverFrac(:);

Pd = zeros(n, 2, K + 1);   PdW = Pd;   Pc = Pd;   Pc_or = Pd;   Pc_max = Pd;   layers = Pd;
for i = 1:n
    for x = 0:1
        for k = 0:K
            % sensors on an UNCOVERED piece and on a camera-COVERED piece: [detection prob, classification accuracy]
            base = zeros(0, 2);
            if x == 1, base(end+1, :) = [v.Pd_DAS, v.Pc_DAS]; end                      %#ok<AGROW>
            if k >= 1, base(end+1, :) = [v.Pd_drone * Ak(k+1), v.Pc_drone]; end        %#ok<AGROW>
            cov = [base; v.Pd_CCTV, v.Pc_CCTV];
            [d0, n0, o0, m0] = piece_stats(base, o.classification_fusion);
            [d1, n1, o1, m1] = piece_stats(cov,  o.classification_fusion);
            Pd(i, x+1, k+1)     = (1 - f(i)) * d0 + f(i) * d1;
            PdW(i, x+1, k+1)    = min(d0, tern(f(i) > 0, d1, inf));
            Pc(i, x+1, k+1)     = ((1 - f(i)) * n0 + f(i) * n1) / max(Pd(i, x+1, k+1), eps);
            Pc_or(i, x+1, k+1)  = ((1 - f(i)) * o0 + f(i) * o1) / max(Pd(i, x+1, k+1), eps);
            Pc_max(i, x+1, k+1) = ((1 - f(i)) * m0 + f(i) * m1) / max(Pd(i, x+1, k+1), eps);
            layers(i, x+1, k+1) = x + (k >= 1) + (f(i) >= 1 - 1e-12);
        end
    end
end
Pc(isnan(Pc)) = 0;   Pc_or(isnan(Pc_or)) = 0;   Pc_max(isnan(Pc_max)) = 0;
need = ones(n, 1);   need(cells.zoneCode == 3) = o.hot_min_layers;
ok = (Pd >= v.AD0 - 1e-12) & (Pc >= v.AC0 - 1e-12) & (layers >= reshape(need, n, 1, 1));
okDet = (Pd >= v.AD0 - 1e-12) & (layers >= reshape(need, n, 1, 1));          % detection + layers only (classification reported as a gap)

M = struct('Pd', Pd, 'PdWorst', PdW, 'Pc', Pc, 'Pc_or', Pc_or, 'Pc_max', Pc_max, 'layers', layers, 'need', need, 'ok', ok, 'okDet', okDet, ...
    'A', Ak, 'K', K, 'A1', A1, 'fusion', o.classification_fusion);
M.bestPc = max(reshape(Pc, n, []), [], 2);
M.gapAC0 = max(0, v.AC0 - M.bestPc);
M.cellOk = any(reshape(ok, n, []), 2);                 % can ANY state satisfy every constraint of the cell?
M.cellOkDet = any(reshape(okDet, n, []), 2);

% drone reach: candidate stations = boundary points every 500 m + interior grid every 2 km (ASSUMED: real dock sites are not public)
sub = 1:100:numel(ring.x);   st = [ring.x(sub) ring.y(sub)];
[gx, gy] = meshgrid(floor(min(ring.x)/2000)*2000 : 2000 : max(ring.x), floor(min(ring.y)/2000)*2000 : 2000 : max(ring.y));
inn = inpolygon(gx(:), gy(:), [ring.x; ring.x(1)], [ring.y; ring.y(1)]);   st = [st; gx(inn) gy(inn)];
sp = 1:10:numel(ring.x);   px = ring.x(sp);   py = ring.y(sp);   pcell = ring.cell(sp);
reach = false(size(st, 1), n);
for s = 1:size(st, 1)
    far = accumarray(pcell, hypot(px - st(s,1), py - st(s,2)), [n 1], @max);   % farthest boundary point of every cell
    reach(s, :) = (far <= v.Cov_drone_m)';
end
M.reach = reach;   M.stationXY = st;
end

function [det, numCorrect, numCorrectOr, numCorrectMax] = piece_stats(S, mode)
% S: rows [d_j, pc_j] of the active sensors on one piece of boundary.
% det = P(detected by any); numCorrect = E[detected and correctly classified] under the chosen fusion rule;
% numCorrectOr / numCorrectMax = same under the optimistic / no-fusion rules (reported as bounds).
m = size(S, 1);
if m == 0, det = 0; numCorrect = 0; numCorrectOr = 0; numCorrectMax = 0; return; end
d = S(:, 1)';   c = S(:, 2)';
det = 1 - prod(1 - d);   numCorrect = 0;   numCorrectOr = 0;   numCorrectMax = 0;
for mask = 1:2^m - 1                                   % non-empty subsets of sensors that detected
    in = bitget(mask, 1:m) == 1;   pr = prod(d(in)) * prod(1 - d(~in));
    numCorrect    = numCorrect    + pr * fuse_classification(c(in), mode);
    numCorrectOr  = numCorrectOr  + pr * fuse_classification(c(in), 'independent_or');
    numCorrectMax = numCorrectMax + pr * fuse_classification(c(in), 'max');
end
end

function t = tern(c, a, b), if c, t = a; else, t = b; end, end
