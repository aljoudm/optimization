function S = asset_zone_summary()
% ASSET_ZONE_SUMMARY  Where do the existing camera sites sit, by zone? (observation for the report; regenerates its numbers)
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
P = load_params();   cells = load_cells(P);   A = place_existing_assets(P, cells);
zn = {'Hot', 'Neutral', 'Cold'};   code = [3 2 1];
inS = A.sites([A.sites.in]);   sc = cells.zoneCode([inS.cell]);
S = table('Size', [3 6], 'VariableTypes', {'string','double','double','double','double','double'}, ...
    'VariableNames', {'zone','cells','sites_in_coverage','share_of_sites_pct','boundary_km','covered_by_cameras_pct'});
for k = 1:3
    in = cells.zoneCode == code(k);
    S(k, :) = {string(zn{k}), nnz(in), nnz(sc == code(k)), 100 * nnz(sc == code(k)) / numel(sc), sum(cells.L_m(in)) / 1e3, 100 * sum(A.coveredLen_m(in)) / sum(cells.L_m(in))};
end
disp(S);
fprintf('Sites in coverage: %d (of which %d in Cold, %d in Hot); mean cell risk of those cells: %.3f (all cells %.3f)\n', ...
    numel(sc), nnz(sc == 1), nnz(sc == 3), mean(cells.risk([inS.cell])), mean(cells.risk));
end
