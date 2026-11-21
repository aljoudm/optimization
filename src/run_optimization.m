function alloc = run_optimization(cells, P, varargin)
% RUN_OPTIMIZATION  Selective DAS / drone allocation by MILP (intlinprog), exact optimum.
%
%   alloc = run_optimization(cells, P)
%   alloc = run_optimization(cells, P, 'Scenario', struct('name','low','N_drone',3), 'Assets', A, 'Model', M, ...
%                            'ProgressSeconds', 600, 'TimeLimit', Inf, 'Verbose', true)
%
%   cells  from load_cells;  P from load_params (P.v values, P.opts modelling options, P.assets existing-asset parameters)
%
% DECISIONS            y_k   interrogator at run k (a site on the boundary, lead-in L_leadin)
%                      z_r   DAS fibre on boundary run r (the boundary is cut into "runs": maximal stretches inside one cell)
%                      a+/a- run r is served by the right / left channel of site k (contiguity: a channel serves the runs
%                            between its site and its last run, all with fibre; reach <= R_DAS - L_leadin)
%                      sigma state (x, k) of every cell: DAS fibre or not, k drones at the serving station
%                      u_sm  drone station s holds m drones (sum of drones <= N_drone of the scenario)
% OBJECTIVE            C_total = C_DAS_int*sum(y) + C_shelter*sum(y) + C_fiber * sum(len_r * z_r)   (existing assets cost 0)
% CONSTRAINTS          every cell is in a state that meets Pd >= AD0, AC0 (see P.opts.ac0_definition) and the layer rule
%                      (Hot >= 2 layers at every point), sigma links to z (x=1 needs fibre on all runs of the cell) and to the
%                      drone stations (k drones need a station within reach holding >= k), drones <= N_drone,
%                      (P_DAS + P_shelter) * sum(y) < P0.
% If no allocation can satisfy every cell (e.g. too few drones) the result is a SOLUTION WITH A GAP:
%   stage 1 minimises the number of cells that violate a requirement, stage 2 minimises C_total with that many violations,
%   stage 3 (tie break) minimises the drones used. The violating cells are listed in alloc.violations and the fewest drones that
%   would leave no violation is reported in alloc.min_drones_no_gap.
% Progress (elapsed time, best solution so far, optimality gap) is printed every ProgressSeconds (default 600 = 10 min).

ip = inputParser;
addParameter(ip, 'Scenario', struct('name', 'custom', 'N_drone', P.v.N_drone_avail));
addParameter(ip, 'Assets', []);   addParameter(ip, 'Model', []);
addParameter(ip, 'ProgressSeconds', 600);   addParameter(ip, 'TimeLimit', Inf);   addParameter(ip, 'Verbose', true);
parse(ip, varargin{:});   o = ip.Results;   scen = o.Scenario;
if ~exist('intlinprog', 'file') || ~license('test', 'Optimization_Toolbox')
    error('run_optimization:toolbox', 'The Optimization Toolbox (intlinprog) is not available. Stop and install / enable it.');
end
t0 = tic;   v = P.v;
if isempty(o.Assets), o.Assets = place_existing_assets(P, cells); end
if isempty(o.Model),  o.Model  = coverage_model(P, cells, o.Assets); end
M = o.Model;   n = cells.n;   K = M.K;   runs = cells.runs;   R = numel(runs.cell);   L = cells.ring.L;

%% ---- DAS channels: which runs a site can serve on each side (arc distance <= R_DAS - lead-in) -----------------------
Rmax = v.R_DAS_m - v.L_leadin_m;   pos = (runs.s0 + runs.s1) / 2;
chain = cell(R, 2);                                      % {k,1} = right chain (increasing arc), {k,2} = left chain
for k = 1:R
    lst = zeros(1, 0);   j = k;
    for t = 1:R-1, j = mod(j, R) + 1;      if mod(runs.s1(j) - pos(k), L) > Rmax + 1e-9, break; end, lst(end+1) = j; end %#ok<AGROW>
    chain{k, 1} = lst;
    lst = zeros(1, 0);   j = k;
    for t = 1:R-1, j = mod(j - 2, R) + 1;  if mod(pos(k) - runs.s0(j), L) > Rmax + 1e-9, break; end, lst(end+1) = j; end %#ok<AGROW>
    chain{k, 2} = lst;
end

%% ---- cell states: every state that meets all requirements, plus (as fallback) every state with DAS ---------------------
S = struct('cell', {}, 'x', {}, 'k', {}, 'viol', {});
for i = 1:n
    for x = 0:1
        for k = 0:K
            okk = M.ok(i, x+1, k+1);
            if okk || x == 1, S(end+1) = struct('cell', i, 'x', x, 'k', k, 'viol', ~okk); end %#ok<AGROW>
        end
    end
end
nS = numel(S);   sCell = [S.cell]';   sX = [S.x]';   sK = [S.k]';   sViol = double([S.viol]');

%% ---- variable indexing --------------------------------------------------------------------------------------------------
nSt = size(M.stationXY, 1);
iy = (1:R)';   iz = R + (1:R)';   off = 2 * R;
nAp = sum(cellfun(@numel, chain(:, 1)));   nAm = sum(cellfun(@numel, chain(:, 2)));
iap = off + (1:nAp)';   off = off + nAp;   iam = off + (1:nAm)';   off = off + nAm;
isg = off + (1:nS)';    off = off + nS;
iu  = reshape(off + (1:nSt * K), K, nSt)';   off = off + nSt * K;          % iu(s, m)
nv = off;

%% ---- constraints (sparse triplets) --------------------------------------------------------------------------------------
rcols = cell(60000, 1);  rvals = cell(60000, 1);  b = zeros(60000, 1);  row = 0;      % inequality rows (grown if needed)
ecols = cell(2000, 1);   evals = cell(2000, 1);    be = zeros(2000, 1);    rowE = 0;       % equality rows
    function addLE(cols, vals, rhs)
        row = row + 1;   rcols{row} = cols(:);   rvals{row} = vals(:);   b(row) = rhs;
    end
    function addEQ(cols, vals, rhs)
        rowE = rowE + 1;   ecols{rowE} = cols(:);   evals{rowE} = vals(:);   be(rowE) = rhs;
    end
servers = cell(R, 1);                                      % for every run: the a-variables that could serve it
cp = iap(1) - 1;  cm = iam(1) - 1;   ap = 0;  am = 0;     % running counters inside the a+ / a- blocks
for k = 1:R
    for side = 1:2
        lst = chain{k, side};
        for t = 1:numel(lst)
            if side == 1, ap = ap + 1; col = cp + ap; else, am = am + 1; col = cm + am; end
            if t == 1, addLE([col, iy(k)], [1 -1], 0);                       % first run of a channel needs the site
            else,      addLE([col, prevCol], [1 -1], 0); end                 % contiguity: run t needs run t-1 of the same channel
            addLE([col, iz(lst(t))], [1 -1], 0);                             % served run carries fibre
            servers{lst(t)}(end+1) = col;                                    %#ok<AGROW>
            prevCol = col;
        end
    end
end
for r = 1:R
    addLE([iy(r), iz(r)], [1 -1], 0);                                        % own run of a site carries fibre
    addLE([iz(r), iy(r), servers{r}], [1, -1, -ones(1, numel(servers{r}))], 0);   % fibre only where a channel serves it
end
for i = 1:n                                                                   % one state per cell
    q = find(sCell == i);   addEQ(isg(q), ones(numel(q), 1), 1);
    for r = cells.cellRuns{i}'                                                % DAS state needs fibre on every run of the cell
        qx = q(sX(q) == 1);   addLE([isg(qx); iz(r)], [ones(numel(qx), 1); -1], 0);
    end
    for kk = 1:K                                                              % k drones need a station within reach holding >= k
        qk = q(sK(q) == kk);   if isempty(qk), continue; end
        st = find(M.reach(:, i))';   cols = zeros(1, 0);
        for s = st, cols = [cols, iu(s, kk:K)]; end                           %#ok<AGROW>
        addLE([isg(qk); cols(:)], [ones(numel(qk), 1); -ones(numel(cols), 1)], 0);
    end
end
for s = 1:nSt, addLE(iu(s, :), ones(1, K), 1); end                            % one drone level per station
droneCols = iu';   droneW = repmat((1:K)', 1, nSt);
addLE(droneCols(:), droneW(:), scen.N_drone);                                 % drones available
addLE(iy, (v.P_DAS + v.P_shelter) * ones(R, 1), v.P0 - 1e-6);                 % power of the new equipment < P0
nBase = row;
lens = cellfun(@numel, rcols(1:row));   Aineq = sparse(repelem((1:row)', lens), vertcat(rcols{1:row}), vertcat(rvals{1:row}), row, nv);   bineq = b(1:row);
lensE = cellfun(@numel, ecols(1:rowE));   Aeq = sparse(repelem((1:rowE)', lensE), vertcat(ecols{1:rowE}), vertcat(evals{1:rowE}), rowE, nv);   beq = be(1:rowE);
lb = zeros(nv, 1);   ub = ones(nv, 1);   intcon = 1:nv;

%% ---- objectives ---------------------------------------------------------------------------------------------------------
fCost = zeros(nv, 1);
fCost(iy) = v.C_DAS_int + v.C_shelter;   fCost(iz) = v.C_fiber_per_km * runs.len / 1e3;
fViol = zeros(nv, 1);   fViol(isg) = sViol;
fDrone = zeros(nv, 1);  fDrone(iu(:)) = repelem(1:K, nSt)' + 1e-3;   % drones used (+ tiny per station)
info = struct();

%% ---- stage 1: fewest violating cells (0 when a complete solution exists) ----------------------------------------------------
[x1, f1, fl1, out1] = solve_stage(fViol, Aineq, bineq, Aeq, beq, lb, ub, intcon, 'stage 1: violations', o, t0);
if fl1 <= 0, error('run_optimization:infeasible', 'No allocation even with violations allowed (exitflag %d).', fl1); end
nViol = round(f1);   info.stage1 = out1;
% ---- stage 2: minimum C_total with that many violations ----------------------------------------------------------------------
A2 = [Aineq; sparse(1, find(fViol), fViol(fViol > 0)', 1, nv)];   b2 = [bineq; nViol + 0.5];
[x2, f2, fl2, out2] = solve_stage(fCost, A2, b2, Aeq, beq, lb, ub, intcon, 'stage 2: C_total', o, t0);
info.stage2 = out2;
% ---- stage 3: tie break, fewest drones at the optimal cost ---------------------------------------------------------------------
A3 = [A2; sparse(1, find(fCost), fCost(fCost > 0)', 1, nv)];   b3 = [b2; f2 + 0.5];
[x3, ~, fl3, out3] = solve_stage(fDrone, A3, b3, Aeq, beq, lb, ub, intcon, 'stage 3: drones', o, t0);
if fl3 > 0, x = x3; else, x = x2; end
info.stage3 = out3;
% ---- if the solution has a gap: the fewest drones that would remove it (no drone limit) --------------------------------------
minDrones = NaN;
if nViol > 0
    Ab = Aineq;   Ab(nBase - 1, :) = 0;   bb = bineq;   bb(nBase - 1) = 0;     % drop the drone-budget row (second to last)
    A4 = [Ab; sparse(1, find(fViol), fViol(fViol > 0)', 1, nv)];   b4 = [bb; 0.5];
    [x4, f4, fl4] = solve_stage(fDrone, A4, b4, Aeq, beq, lb, ub, intcon, 'extra: fewest drones, no gap', o, t0);
    if fl4 > 0, minDrones = round(f4 - 1e-3 * nnz(round(x4(iu(:))))); end
end

%% ---- unpack ---------------------------------------------------------------------------------------------------------------
x = round(x);
alloc = struct();
alloc.scenario = scen;   alloc.params = P.v;
y = x(iy) == 1;   z = x(iz) == 1;
alloc.interrogators.run = find(y);   alloc.interrogators.arc_m = pos(y);   alloc.n_interrogators = nnz(y);
[~, pj] = min(abs(cells.ring.s(:)' - pos(y)), [], 2);
alloc.interrogators.lat = cells.ring.lat0 + cells.ring.y(pj) / cells.ring.ky;   alloc.interrogators.lon = cells.ring.lon0 + cells.ring.x(pj) / cells.ring.kx;
alloc.fibre_runs = z;   alloc.fibre_km = sum(runs.len(z)) / 1e3;
alloc.cost.interrogators = nnz(y) * v.C_DAS_int;   alloc.cost.shelters = nnz(y) * v.C_shelter;   alloc.cost.fibre = v.C_fiber_per_km * alloc.fibre_km;
alloc.C_total = alloc.cost.interrogators + alloc.cost.shelters + alloc.cost.fibre;
alloc.power.interrogators_W = nnz(y) * v.P_DAS;   alloc.power.shelters_W = nnz(y) * v.P_shelter;
alloc.power.total_W = alloc.power.interrogators_W + alloc.power.shelters_W;   alloc.power.P0_W = v.P0;   alloc.power.within_P0 = alloc.power.total_W < v.P0;
sel = find(round(x(isg)) == 1);   st = S(sel);
cellX = zeros(n, 1);  cellK = zeros(n, 1);  cellViol = false(n, 1);
for q = 1:numel(st), cellX(st(q).cell) = st(q).x;  cellK(st(q).cell) = st(q).k;  cellViol(st(q).cell) = st(q).viol; end
U = reshape(x(iu(:)), nSt, K);   nDr = (U * (1:K)')';                          % drones at every candidate station
alloc.drones.station_xy = M.stationXY(nDr > 0, :);   alloc.drones.n = nDr(nDr > 0);   alloc.drones.total = sum(nDr);   alloc.drones.stations = nnz(nDr);
cellStation = zeros(n, 1);
for i = 1:n
    if cellK(i) >= 1, s = find(M.reach(:, i) & nDr' >= cellK(i), 1);  if ~isempty(s), cellStation(i) = s; end, end
end
alloc.cells = table((1:n)', string(cells.zone), cellX, cellK, cellStation, M.A(cellK + 1)', 'VariableNames', {'cell','zone','DAS','drones_serving','station','drone_availability'});
lin = @(a) sub2ind(size(M.Pd), (1:n)', cellX + 1, cellK + 1);
alloc.cells.Pd = M.Pd(lin());   alloc.cells.Pc_system = M.Pc(lin());   alloc.cells.Pc_DAS = v.Pc_DAS * cellX;
alloc.cells.layers = M.layers(lin());   alloc.cells.layers_needed = M.need;   alloc.cells.violates = cellViol;
alloc.cells.length_m = cells.L_m;   alloc.cells.fibre_cost = v.C_fiber_per_km * cells.L_m / 1e3 .* cellX;
alloc.cells.camera_fraction = o.Assets.coverFrac;
alloc.violations = find(cellViol);   alloc.n_violations = numel(alloc.violations);
alloc.feasible = alloc.n_violations == 0;
alloc.min_drones_no_gap = minDrones;
alloc.system_accuracy_ok = nnz(alloc.cells.Pc_system >= v.AC0 - 1e-12);          % extra result: whole-system accuracy, conditional on detection
alloc.solver = info;   alloc.elapsed_s = toc(t0);   alloc.options = P.opts;
if o.Verbose
    fprintf('[%s] scenario %s: C_total %.0f SAR | interrogators %d | fibre %.1f km | DAS cells %d/%d | drones %d at %d stations | violations %d | P %.0f W (P0 %.0f W) | %.0f s\n', ...
        datestr(now, 'HH:MM:SS'), scen.name, alloc.C_total, alloc.n_interrogators, alloc.fibre_km, nnz(cellX), n, alloc.drones.total, alloc.drones.stations, alloc.n_violations, alloc.power.total_W, v.P0, alloc.elapsed_s);
end
end

function [x, f, flag, out] = solve_stage(f0, A, b, Aeq, beq, lb, ub, intcon, label, o, t0)
% One intlinprog call with a progress printer (every o.ProgressSeconds) and the stage result.
st = struct('last', toc(t0), 'label', label, 'o', o, 't0', t0);
opt = optimoptions('intlinprog', 'Display', 'off', 'RelativeGapTolerance', 1e-9, 'AbsoluteGapTolerance', 1e-6, ...
    'MaxTime', o.TimeLimit, 'OutputFcn', @(xx, ov, state) progress(ov, state, st));
[x, f, flag, out] = intlinprog(f0, intcon, A, b, Aeq, beq, lb, ub, opt);
if o.Verbose
    if isempty(f), f = NaN; end
    fprintf('  %-32s exitflag %d | objective %.6g | gap %.3g%% | %.1f s\n', label, flag, f, 100 * getfield_def(out, 'relativegap', NaN), toc(t0) - st.last);
end
end

function stop = progress(ov, state, st)
% Prints elapsed time, best solution so far and optimality gap every st.o.ProgressSeconds (10 minutes by default).
persistent tLast;
stop = false;
if strcmp(state, 'init'), tLast = tic; return; end
if ~isempty(tLast) && toc(tLast) >= st.o.ProgressSeconds && st.o.Verbose
    fprintf('    [%s | %s] elapsed %.0f s | best solution %.6g | best bound %.6g | gap %.3g%%\n', datestr(now, 'HH:MM:SS'), st.label, toc(st.t0), ...
        ov.fval, ov.dualbound, 100 * ov.relativegap);
    tLast = tic;
end
end

function s = getfield_def(st, f, d), if isfield(st, f), s = st.(f); else, s = d; end, end
function t = tern(c, a, b), if c, t = a; else, t = b; end, end
