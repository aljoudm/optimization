function make_clean_map()
% MAKE_CLEAN_MAP  Medium-scenario optimal map WITHOUT the title and the warning line (the caption in the report explains it):
% report/s3_optimization_map_clean.pdf (vector). The warning about the cabins (C_shelter = 0) belongs in the caption.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
P = load_params();   P.assets.allow_filler = false;   cells = load_cells(P);
S = load(fullfile(here, '..', 'results', 'allocs.mat'), 'allocs');   allocs = S.allocs;
k = find(cellfun(@(a) strcmp(a.caseInfo.label, 'grid') && strcmp(a.caseInfo.interdiction, 'real22') && strcmp(a.scenario.name, 'medium') && abs(a.caseInfo.Pd_DAS - 0.969) < 1e-9 ...
    && abs(a.caseInfo.Pc_DAS - 0.983) < 1e-9 && a.caseInfo.T_resp == 300 && a.caseInfo.A_ver_min == 0.9 && a.caseInfo.C_fiber_factor == 1, allocs), 1);
al = allocs{k};
lay = drone_layout(cells, P, al.drones.station_xy, al.drones.n(:));
f = figure('Color', 'w', 'Position', [60 40 980 720], 'Visible', 'off');   ax = axes(f);
draw_allocation_map(ax, cells, P, al, lay, al.drones.station_xy, al.drones.n(:), '', '');
exportgraphics(f, fullfile(here, '..', 'report', 's3_optimization_map_clean.pdf'), 'ContentType', 'vector');   close(f);
fprintf('Saved report/s3_optimization_map_clean.pdf (medium scenario, Hot verified %d/36)\n', nnz(lay.verified & cells.zoneCode == 3));
end
