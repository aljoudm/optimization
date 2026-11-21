function test_load_params()
% TEST_LOAD_PARAMS  Independent checks of load_params against numbers written by hand from the table / README.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'src'));
P = load_params();
v = P.v;  t = P.tag;
tol = 1e-12;
chk = @(name, expected) assert(abs(v.(name) - expected) < tol, 'FAIL %s: %.10g vs %.10g', name, v.(name), expected);

% (1) values typed by hand from the table rows
chk('AD0', 0.98);        chk('AC0', 0.98);       chk('R_DAS_m', 20000);   chk('N_ch_active', 2);
chk('N_ch_DAS', 4);      chk('P_DAS', 170);      chk('C_DAS_int', 636000);
chk('Cov_CCTV_m', 100);  chk('P_CCTV', 20);      chk('Pd_CCTV', 0.909);   chk('Pc_CCTV', 0.85);
chk('Cov_drone_m', 11000); chk('P_drone', 1500); chk('T_drone_min', 47);  chk('Pd_drone', 0.885);
chk('cell_size_m', 1000); chk('R_hot', 0.464);   chk('R_cold', 0.263);    chk('L_boundary_m', 145473.5);
chk('N_drone_avail', 3); chk('L_leadin_m', 0);
assert(isequal(v.N_cells_zone, [36 56 47]), 'zone counts');
assert(abs(sum(v.L_zone_km) - 145.5) < 1e-9, 'zone lengths must sum to 145.5 km');

% (2) derived values recomputed independently
assert(v.N_DAS_int_full == 4, 'N_DAS_int = ceil(145.4735/40) = 4');
assert(v.P0 == 1455*20, 'P0 = ceil(1454.735)*20 = 29100 W');
assert(abs(v.A_drone_1 - 47/(47+60)) < tol, 'availability of one drone');
% full-DAS reference cost (hand): 4*636000 = 2,544,000 ; 145.4735*60700 = 8,830,241.45 ; total 11,374,241.45
fullCost = 4*636000 + 145.4735*60700;
assert(abs(fullCost - 11374241.45) < 0.01, 'hand-computed reference cost %.2f', fullCost);

% (3) every value has a tag from the allowed list, and a source text
allowed = {'TABLE','TABLE-DERIVED','ASSUMED','PLACEHOLDER','DERIVED-PROPOSED'};
f = fieldnames(v);
for k = 1:numel(f)
    assert(isfield(t, f{k}) && ismember(t.(f{k}), allowed), 'missing / bad tag for %s', f{k});
    assert(isfield(P.src, f{k}) && ~isempty(P.src.(f{k})), 'missing source for %s', f{k});
end
% (4) placeholders are tagged as such and the table TBD cells are really TBD
for s = {'Pd_DAS','Pc_DAS','Pc_drone','P_shelter','T_charge_min','C_shelter','C_fiber_per_km'}
    assert(strcmp(t.(s{1}), 'PLACEHOLDER'), '%s must be PLACEHOLDER', s{1});
    assert(isfield(P.range, s{1}), 'no sensitivity range for %s', s{1});
end
assert(isnan(v.Pd_PS_value) && isnan(v.Cov_PS_value), 'Point-sensor values must stay NaN');
% (5) probabilities are in [0,1]
for s = {'AD0','AC0','Pd_CCTV','Pc_CCTV','Pd_drone','Pd_DAS','Pc_DAS','Pc_drone'}
    assert(v.(s{1}) > 0 && v.(s{1}) < 1, '%s out of range', s{1});
end
fprintf('test_load_params: all checks passed (%d values, %d tagged PLACEHOLDER).\n', numel(f), ...
    sum(structfun(@(x) strcmp(x, 'PLACEHOLDER'), t)));
end
