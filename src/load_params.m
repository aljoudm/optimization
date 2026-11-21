function P = load_params(tsvFile)
% LOAD_PARAMS  Single parameter source of the optimization module.
%
%   P = load_params()            reads ../parameters_reviewed.tsv (relative to this file)
%   P = load_params(tsvFile)     reads another copy of the table
%
%   Every number comes from parameters_reviewed.tsv EXCEPT the two clearly marked blocks below:
%     (1) ASSUMED  block : values the project brief / DAS-map README state but the table lacks.
%     (2) PLACEHOLDER block : values the table marks TBD (or lacks entirely). They are NOT real.
%   Each value in P.v has a tag in P.tag and a human-readable source in P.src.
%
%   Internal units (P.v): length [m], power [W], cost [SAR], time [min], probabilities in [0,1].
%
%   Output fields
%     P.v        flat struct of values used by the model
%     P.tag      same field names -> 'TABLE' | 'TABLE-DERIVED' | 'ASSUMED' | 'PLACEHOLDER' | 'DERIVED-PROPOSED'
%     P.src      same field names -> source / note text
%     P.range    sensitivity ranges for PLACEHOLDER (and doubtful) values, [lo hi]
%     P.raw      all table rows (symbol, value text, unit, source, status, note)
%     P.opts     modelling switches (defaults = recommended options, can be overridden by the caller)

if nargin < 1 || isempty(tsvFile)
    tsvFile = fullfile(fileparts(mfilename('fullpath')), '..', 'parameters_reviewed.tsv');
end
assert(isfile(tsvFile), 'load_params:noFile', 'Parameter table not found: %s', tsvFile);

%% ---- 1. read the table (tab separated, UTF-8, header row, 9 columns) -------------------------------------
txt   = fileread(tsvFile, 'Encoding', 'UTF-8');
lines = splitlines(string(txt));
lines = lines(strlength(strtrim(lines)) > 0);
lines(1) = [];                                         % header
raw = struct('sym', {}, 'value', {}, 'unit', {}, 'source', {}, 'status', {}, 'note', {});
seen = containers.Map('KeyType', 'char', 'ValueType', 'double');
for r = 1:numel(lines)
    c = split(lines(r), char(9));
    assert(numel(c) == 9, 'load_params:badRow', 'Row %d of the table has %d columns (expected 9).', r + 1, numel(c));
    sym = char(strtrim(c(1)));
    if isKey(seen, sym), seen(sym) = seen(sym) + 1; sym = sprintf('%s_dup%d', sym, seen(sym));
    else,                seen(sym) = 1; end
    raw(end+1) = struct('sym', sym, 'value', strtrim(c(4)), 'unit', strtrim(c(5)), 'source', strtrim(c(6)), ...
                        'status', strtrim(c(8)), 'note', strtrim(c(9))); %#ok<AGROW>
end
names = {raw.sym};
row = @(s) raw(strcmp(names, s));                       % table row by symbol (errors below if missing)
num = @(s) parse_number(row(s).value);                   % numeric value of a row, NaN if TBD / not a number

v = struct();  tag = struct();  src = struct();  rng = struct();
    function put(name, value, tg, source)               % register one value with its tag and source
        v.(name) = value;  tag.(name) = tg;  src.(name) = char(source);
    end
    function need(s)
        assert(any(strcmp(names, s)), 'load_params:missingRow', 'Symbol %s is not in the table.', s);
    end

%% ---- 2. values taken directly from the table --------------------------------------------------------------
cellfun(@need, {'AD0','R_DAS','N_ch_DAS','N_ch_active','P_DAS','C_DAS_int','C_fiber','L_boundary','L_leadin', ...
    'Cov_CCTV','P_CCTV','Pd_CCTV','Pc_CCTV','C_CCTV','Cov_drone','P_drone','T_drone','Pd_drone','C_drone', ...
    'N_drone_avail','cell_size','R_hot','R_cold','N_cells','L_zone','P0','P_shelter','Pd_DAS','Pc_DAS','Pc_drone', ...
    'Pd_PS','Pc_PS','Cov_PS','P_PS','C_PS'});

put('AD0',          num('AD0')/100,        'TABLE', 'AD0, status approved');
put('R_DAS_m',      num('R_DAS')*1e3,      'TABLE', 'R_DAS (20 km option, FEBUS A1-NEO datasheet)');
put('N_ch_DAS',     num('N_ch_DAS'),       'TABLE', 'N_ch_DAS (installed channels per unit)');
put('N_ch_active',  num('N_ch_active'),    'TABLE', 'N_ch_active (one channel per direction along the boundary)');
put('P_DAS',        num('P_DAS'),          'TABLE', 'P_DAS (datasheet, approximate ~170 W)');
put('C_DAS_int',    num('C_DAS_int'),      'TABLE', 'C_DAS_int: 150,000 EUR x 4.24 SAR/EUR (FX ASSUMED, provisional)');
put('L_leadin_m',   num('L_leadin'),       'TABLE', 'L_leadin (cabin assumed ON the boundary)');
put('Cov_CCTV_m',   num('Cov_CCTV'),       'TABLE', 'Cov_CCTV (no written source in the table)');
put('P_CCTV',       num('P_CCTV'),         'TABLE', 'P_CCTV (Hikvision / Dahua datasheets)');
put('Pd_CCTV',      num('Pd_CCTV')/100,    'TABLE', 'Pd_CCTV (Park et al., Table 3)');
put('Pc_CCTV',      num('Pc_CCTV')/100,    'TABLE', 'Pc_CCTV (Jebur et al., Table 5)');
put('Cov_drone_m',  num('Cov_drone')*1e3,  'TABLE', 'Cov_drone = operating RADIUS (proposal)');
put('P_drone',      num('P_drone'),        'TABLE', 'P_drone (peak figures, slightly pessimistic; proposal)');
put('T_drone_min',  num('T_drone'),        'TABLE', 'T_drone flight time per charge (proposal)');
put('Pd_drone',     num('Pd_drone')/100,   'TABLE', 'Pd_drone (average of 3 UAV thermal studies; proposal)');
put('cell_size_m',  num('cell_size'),      'TABLE', 'cell_size (DAS-map, provisional)');
put('R_hot',        num('R_hot'),          'TABLE', 'R_hot (DAS-map k-means, provisional)');
put('R_cold',       num('R_cold'),         'TABLE', 'R_cold (DAS-map k-means, provisional)');
% L_boundary: the table column is rounded (145.5 km); the exact haversine value is quoted in the review note.
tok = regexp(char(row('L_boundary').note), '([\d,]+\.\d+)\s*m', 'tokens', 'once');
assert(~isempty(tok), 'load_params:LB', 'Exact L_boundary not found in the L_boundary note.');
put('L_boundary_m', str2double(strrep(tok{1}, ',', '')), 'TABLE', 'L_boundary exact value quoted in the review note (provisional: Arafat part)');
assert(abs(v.L_boundary_m/1e3 - num('L_boundary')) < 0.05, 'load_params:LB2', 'L_boundary note and column disagree.');
% zone counts / lengths ("139 (36 / 56 / 47)" and "39.4 / 59.0 / 47.1"), order Hot / Neutral / Cold
n3 = sscanf(strrep(strrep(char(row('N_cells').value), '/', ' '), '(', ' '), '%f')';   % [139 36 56 47]
assert(numel(n3) == 4 && n3(1) == sum(n3(2:4)), 'load_params:Ncells', 'N_cells row is inconsistent.');
put('N_cells_zone', n3(2:4), 'TABLE', 'N_cells row, order Hot / Neutral / Cold');
l3 = sscanf(strrep(char(row('L_zone').value), '/', ' '), '%f')';
put('L_zone_km', l3, 'TABLE', 'L_zone row, order Hot / Neutral / Cold');
% reference costs (existing assets cost us ZERO in the optimization; kept for comparison only)
put('C_CCTV_ref',  num('C_CCTV'),  'TABLE', 'C_CCTV, reference only (existing asset: zero cost for us)');
put('C_drone_ref', num('C_drone'), 'TABLE', 'C_drone, reference only (existing asset: zero cost for us)');
put('C_existing',  0,              'ASSUMED', 'Project rule: existing CCTV / drones / Point-Sensor Networks have zero capital cost for us');

%% ---- 3. ASSUMED block (stated by the brief / DAS-map README, not a table value) ---------------------------
put('AC0',          0.98, 'ASSUMED', 'AC0 = AD0 = 98% (project brief and DAS-map README "project target, approved"); the table has no AC0 row');
put('N_drone_avail', num('N_drone_avail'), 'ASSUMED', 'N_drone_avail = 3 (table: ILLUSTRATIVE; real number not public)');
put('N_CCTV_illustrative', 16, 'ASSUMED', 'Loc_existing: 16 cameras, ILLUSTRATIVE (one per major road crossing, rest < 500 m from roads)');
put('road_snap_km', 0.5, 'ASSUMED', 'Loc_existing: extra cameras only within 500 m of a road (from the table note)');

%% ---- 4. PLACEHOLDER block: NOT real values. Edit HERE only. Each one is swept in the sensitivity study. ---
PH.Pd_DAS     = struct('value', 0.95, 'range', [0.90 0.99], 'why', 'Pd_DAS is TBD (comes from the DAS-Simulation Monte Carlo)');
PH.Pc_DAS     = struct('value', 0.90, 'range', [0.80 0.98], 'why', 'Pc_DAS is TBD (comes from the Stage-1 AI classifier)');
PH.Pc_drone   = struct('value', 0.85, 'range', [0.80 0.95], 'why', 'Pc_drone is TBD in the table');
PH.P_shelter  = struct('value', 1000, 'range', [500 3000],  'why', 'P_shelter is TBD (air-conditioning power per cabin, W)');
PH.T_charge_min = struct('value', 60,  'range', [30 120],   'why', 'drone recharge time is NOT in the table at all (needed for availability)');
PH.C_shelter  = struct('value', 0,     'range', [0 300000], 'why', 'cabin cost is NOT in the table; 0 means "not costed yet"');
% C_fiber has a number in the table but the reviewers flag it as extremely doubtful -> treated as a placeholder-grade value.
PH.C_fiber_per_km = struct('value', num('C_fiber'), 'range', [20000 120000], ...
                           'why', 'table value 60,700 SAR/km has no source and is flagged extremely doubtful');
fn = fieldnames(PH);
for k = 1:numel(fn)
    put(fn{k}, PH.(fn{k}).value, 'PLACEHOLDER', PH.(fn{k}).why);
    rng.(fn{k}) = PH.(fn{k}).range;
end
% Point-Sensor Networks: every value is TBD in the table -> excluded from the base case (no data, no locations).
put('use_PS', false, 'ASSUMED', 'Point-Sensor Networks excluded from the base case: all their values are TBD; hooks kept in the code');
for s = {'Cov_PS','P_PS','Pd_PS','Pc_PS','C_PS'}
    put([s{1} '_value'], num(s{1}), 'PLACEHOLDER', [s{1} ' is TBD in the table (NaN = not available)']);
end

%% ---- 5. derived quantities (formula and its inputs stated) ------------------------------------------------
put('N_DAS_int_full', ceil(v.L_boundary_m / (v.N_ch_active * v.R_DAS_m)), 'TABLE-DERIVED', ...
    'ceil(L_boundary / (N_ch_active * R_DAS)) = 4 (table row N_DAS_int); upper-bound check only, the MILP decides the number');
% P0: the table proposes "CCTV-only system covering the same length" (row P0 note). It is a proposal, not a source.
put('P0', ceil(v.L_boundary_m / v.Cov_CCTV_m) * v.P_CCTV, 'DERIVED-PROPOSED', ...
    'P0 = ceil(L_boundary / Cov_CCTV) * P_CCTV, the method proposed in the P0 row note (table value itself is TBD)');
rng.P0 = [0.1 1] * v.P0;
put('A_drone_1', v.T_drone_min / (v.T_drone_min + v.T_charge_min), 'DERIVED-PROPOSED', ...
    'availability of ONE drone = T_drone / (T_drone + T_charge); T_charge is a PLACEHOLDER');

%% ---- 6. modelling options (defaults = the recommendations; the caller may override) ---------------------------
opts = struct();
opts.classification_fusion = 'weighted_vote';  % 'weighted_vote' | 'max' | 'independent_or' (optimistic upper bound)
opts.hot_layers_rule       = 'pointwise';      % each point of a Hot cell must be covered by >= 2 layers
opts.hot_min_layers        = 2;
opts.relax_cold            = false;            % true: scenario with a relaxed target in Cold cells (decision pending)
opts.drone_time_share      = false;            % true: split a drone's watch time over the cells it serves (sensitivity)

P = struct('v', v, 'tag', tag, 'src', src, 'range', rng, 'raw', raw, 'opts', opts, 'tsvFile', string(tsvFile));
end

function x = parse_number(s)
% First plain number in the text (unicode minus allowed); NaN for TBD or non-numeric text.
s = strrep(char(s), char(8722), '-');
if isempty(regexp(s, '^\s*-?\d', 'once')), x = NaN; return; end
x = sscanf(s, '%f', 1);
if isempty(x), x = NaN; end
end
