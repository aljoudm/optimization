function summarize_runs()
% SUMMARIZE_RUNS  Reads results/allocs.mat and prints how the drone / DAS allocation changes with Pd_DAS and the scenario.
here = fileparts(mfilename('fullpath'));   addpath(fullfile(here, '..', 'src'));
S = load(fullfile(here, '..', 'results', 'allocs.mat'));   al = S.allocs;
P = load_params();   cells = load_cells(P);   ring = cells.ring;
pick = @(inter, sc, pd, pc, cf) find(cellfun(@(a) strcmp(a.caseInfo.interdiction, inter) && strcmp(a.scenario.name, sc) && abs(a.caseInfo.Pd_DAS - pd) < 1e-9 ...
    && abs(a.caseInfo.Pc_DAS - pc) < 1e-9 && a.caseInfo.C_fiber_factor == cf, al), 1);
zn = {'Cold', 'Neutral', 'Hot'};
for sc = {'low', 'medium', 'high'}
    for pd = [0.969 0.98 1.0]
        a = al{pick('with_filler32', sc{1}, pd, 0.983, 1)};   T = a.cells;
        served = T.drones_serving >= 1;
        z = arrayfun(@(k) nnz(served & cells.zoneCode == k), 1:3);   zt = arrayfun(@(k) nnz(cells.zoneCode == k), 1:3);
        fprintf('\n%-6s Pd_DAS %.3f: DAS cells %d/139 | interrogators %d | drones %d at %d stations | cells with a drone layer: Hot %d/%d, Neutral %d/%d, Cold %d/%d | below requirement: %d\n', ...
            sc{1}, pd, nnz(T.DAS), a.n_interrogators, a.drones.total, a.drones.stations, z(3), zt(3), z(2), zt(2), z(1), zt(1), a.n_violations);
        for s = 1:size(a.drones.station_xy, 1)
            fprintf('    drone station %d: lat %.4f lon %.4f, %d drone(s)\n', s, ring.lat0 + a.drones.station_xy(s, 2) / ring.ky, ring.lon0 + a.drones.station_xy(s, 1) / ring.kx, a.drones.n(s));
        end
        if a.n_violations > 0
            fprintf('    cells below AD0: %s (zones %s), Pd %s; fewest drones with no gap: %d\n', mat2str(a.violations'), strjoin(string(cells.zone(a.violations))', ','), mat2str(T.Pd(a.violations)', 4), a.min_drones_no_gap);
        end
    end
end
a = al{pick('with_filler32', 'medium', 0.969, 0.983, 1)};   b = al{pick('with_filler32', 'medium', 0.969, 0.993, 1)};
fprintf('\nSystem accuracy (extra result, conditional on detection), medium / Pd 0.969: Pc_DAS 0.983 -> min %.4f max %.4f ; Pc_DAS 0.993 -> min %.4f max %.4f\n', min(a.cells.Pc_system), max(a.cells.Pc_system), min(b.cells.Pc_system), max(b.cells.Pc_system));
x = al{pick('real22', 'medium', 0.969, 0.983, 1)};   y = al{pick('with_filler32', 'medium', 0.969, 0.983, 1)};
fprintf('Interdiction 22 real vs 32 with filler (medium, Pd 0.969): C_total %.2f vs %.2f; DAS cells %d vs %d; drones %d vs %d; identical station set: %d\n', x.C_total, y.C_total, nnz(x.cells.DAS), nnz(y.cells.DAS), x.drones.total, y.drones.total, isequal(sortrows(x.drones.station_xy), sortrows(y.drones.station_xy)));
end
