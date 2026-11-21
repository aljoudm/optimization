function test_cells_assets()
% TEST_CELLS_ASSETS  Checks load_cells (order along the boundary) and place_existing_assets against DAS-map's own numbers.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
P = load_params();
cells = load_cells(P);

% (1) the README numbers: 139 cells = 36 Hot / 56 Neutral / 47 Cold, boundary km per zone 39.4 / 59.0 / 47.1
assert(cells.n == 139, 'cell count');
nz = [sum(cells.zoneCode == 3) sum(cells.zoneCode == 2) sum(cells.zoneCode == 1)];
assert(isequal(nz, P.v.N_cells_zone), 'zone counts %s', mat2str(nz));
kmz = [sum(cells.L_m(cells.zoneCode == 3)) sum(cells.L_m(cells.zoneCode == 2)) sum(cells.L_m(cells.zoneCode == 1))] / 1e3;
assert(all(abs(kmz - P.v.L_zone_km) < 0.05), 'zone km %s', mat2str(kmz, 4));

% (2) runs tile the whole ring without gaps or overlaps, in order
r = cells.runs;
assert(abs(r.s0(1)) < 1e-9 && abs(r.s1(end) - cells.ring.L) < 1e-6, 'runs must start at 0 and end at L');
assert(all(abs(r.s0(2:end) - r.s1(1:end-1)) < 1e-6), 'runs must be contiguous');
assert(all(r.cell(2:end) ~= r.cell(1:end-1)), 'neighbouring runs must belong to different cells');
assert(abs(sum(r.len) - cells.ring.L) < 1e-6, 'run lengths sum');
% (3) per-cell run lengths add up to the cell length (independent accumulation)
for i = 1:cells.n
    assert(abs(sum(r.len(cells.cellRuns{i})) - cells.L_m(i)) < 1e-6, 'cell %d length', i);
end
% (4) ring length: planar local frame vs the haversine value of the table (the two differ slightly; report, bound at 1%)
dev = (cells.ring.L - P.v.L_boundary_m) / P.v.L_boundary_m;
assert(abs(dev) < 0.01, 'planar ring length deviates %.3f%% from L_boundary', 100*dev);
multi = nnz(cellfun(@numel, cells.cellRuns) > 1);
fprintf('cells: %d (Hot %d / Neutral %d / Cold %d); ring %.1f m (table %.1f m, %+.3f%%); runs %d; cells with >1 run: %d\n', ...
    cells.n, nz, cells.ring.L, P.v.L_boundary_m, 100*dev, numel(r.cell), multi);

% (5) cameras
A = place_existing_assets(P, cells, [], true);
Aa = P.assets;   T = A.checkpointTable;   kinds = string({A.sites.kind});
assert(height(T) == 6 && all(T.ways_matched > 0), 'every checkpoint must match an OSM road');
assert(~T.in_coverage(strcmp(T.checkpoint, 'Al-Laith')), 'Al-Laith must be outside the coverage (project lead)');
assert(nnz(kinds == "checkpoint") == 6, '6 checkpoint sites');
assert(nnz(kinds ~= "checkpoint") == Aa.n_interdiction, 'interdiction + filler sites = %d', Aa.n_interdiction);
% Al-Sharaye (ref 40, east) and Al-Hada (ref 15) must be different crossings, far apart; Sharaye must lie on ref 40
iS = strcmp(T.checkpoint, 'Al-Sharaye (Taif road via Al-Sail)');   iH = strcmp(T.checkpoint, 'Al-Hada (Taif road via Al-Hada)');
assert(T.osm_ref(iS) == "40" && T.osm_ref(iH) == "15", 'Sharaye must be on ref 40 and Hada on ref 15');
dSH = hypot((T.lon(iS) - T.lon(iH)) * cells.ring.kx, (T.lat(iS) - T.lat(iH)) * cells.ring.ky);
assert(dSH > 5000, 'Sharaye and Hada must not share a crossing (%.0f m apart)', dSH);
% interdiction: real crossings only (no filler) vs completed with filler
Pn = P;  Pn.assets.allow_filler = false;   An = place_existing_assets(Pn, cells);
assert(nnz(string({An.sites.kind}) == "interdiction") == A.nInterdictionPool && ~any(string({An.sites.kind}) == "filler"), 'no-filler case');
fprintf('interdiction cases: %d real crossings only -> covered %.0f m (%d cells); %d (with %d filler) -> covered %.0f m (%d cells)\n', ...
    A.nInterdictionPool, sum(An.coveredLen_m), nnz(An.coveredLen_m > 0), Aa.n_interdiction, nnz(kinds == "filler"), sum(A.coveredLen_m), nnz(A.coveredLen_m > 0));
assert(all([A.sites.cell] >= 1 & [A.sites.cell] <= cells.n), 'site cells');
nIn = nnz([A.sites.in]);
% coverage cannot exceed one footprint per "in" site, and must not be zero
assert(sum(A.coveredLen_m) <= nIn * (Aa.footprint_m + 10) + 1e-6 &&sum(A.coveredLen_m) > 0.5 * nIn * Aa.footprint_m, 'covered length');
assert(all(A.coverFrac >= 0 & A.coverFrac <= 1), 'cover fraction');
% camera counts do NOT multiply coverage (all cameras of a site watch the same lanes)
P2 = P;  P2.assets.cams_per_checkpoint = 100;   P2.assets.cams_per_interdiction = 1;
A2 = place_existing_assets(P2, cells);
assert(isequal(A2.coveredLen_m, A.coveredLen_m), 'camera counts must not change the covered length');
% the footprint parameter does
P3 = P;  P3.assets.footprint_m = 4 * P.assets.footprint_m;
A3 = place_existing_assets(P3, cells);
assert(sum(A3.coveredLen_m) > 3 * sum(A.coveredLen_m) * 0.9, 'footprint x4 must roughly quadruple the covered length');
fprintf('cameras counted in cells: %d (checkpoints %d x %d + interdiction); coverage x4 footprint: %.0f m\n', sum(A.nCamsCell), nnz([A.sites.in] & kinds == "checkpoint"), Aa.cams_per_checkpoint, sum(A3.coveredLen_m));
hot = cells.zoneCode == 3;   sc = [A.sites.cell];   si = [A.sites.in];
fprintf('sites in coverage by zone: Hot %d / Neutral %d / Cold %d\n', sum(hot(sc(si))), sum(cells.zoneCode(sc(si)) == 2), sum(cells.zoneCode(sc(si)) == 1));
fprintf('test_cells_assets: all checks passed.\n');
end
