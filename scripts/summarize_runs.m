function summarize_runs()
% SUMMARIZE_RUNS  Prints the result tables from results/summary_runs.csv (model of 22 Nov: DAS + cameras detect, drones verify).
here = fileparts(mfilename('fullpath'));
T = readtable(fullfile(here, '..', 'results', 'summary_runs.csv'), 'TextType', 'string');   T.Properties.VariableNames{1} = 'kind';   % 'case' is a reserved word
base = T(T.kind == "grid" & T.interdiction_case == "real22", :);
fprintf('\n== Grid: scenario x Pd_DAS (22 real interdiction crossings, T_resp 300 s, A_ver_min 0.9) ==\n');
disp(base(:, {'scenario','N_drone','Pd_DAS','C_total_SAR','fibre_share','interrogators','DAS_cells','drones_used','drone_stations','Hot_verified','cells_below_AD0','max_Pd_gap','fewest_drones_all_Hot_verified','sumA_ver_Hot'}));
fprintf('\n== 22 real crossings vs 32 with filler (same scenario / Pd_DAS) ==\n');
a = T(T.kind == "grid" & T.interdiction_case == "real22", :);   b = T(T.kind == "grid" & T.interdiction_case == "with_filler32", :);
fprintf('max |C_total difference| %.4f SAR; DAS cells equal: %d; Hot verified equal: %d; drones equal: %d\n', max(abs(a.C_total_SAR - b.C_total_SAR)), isequal(a.DAS_cells, b.DAS_cells), isequal(a.Hot_verified, b.Hot_verified), isequal(a.drones_used, b.drones_used));
fprintf('\n== Sensitivity: T_resp (fewest drones that verify every Hot cell, no drone limit) ==\n');
t = T(T.kind == "T_resp" | (T.kind == "grid" & T.interdiction_case == "real22" & T.N_drone == 6), :);
t = t(t.Pd_DAS == 0.969 & t.N_drone == 6, :);   [~, o] = sort(t.T_resp_s);   t = t(o, :);
disp(t(:, {'T_resp_s','Hot_verified','Hot_cells','fewest_drones_all_Hot_verified','drones_used','drone_stations'}));
for sc = ["low" "medium" "high"]
    s = T(T.kind == "T_resp" & T.scenario == sc & T.Pd_DAS == 0.969, :);   g = T(T.kind == "grid" & T.interdiction_case == "real22" & T.scenario == sc & T.Pd_DAS == 0.969, :);
    s = [g; s];   [~, o] = sort(s.T_resp_s);   s = s(o, :);
    fprintf('  %-6s: T_resp %s -> Hot verified %s of 36, fewest drones for all Hot %s\n', sc, mat2str(s.T_resp_s'), mat2str(s.Hot_verified'), mat2str(s.fewest_drones_all_Hot_verified'));
end
fprintf('\n== Sensitivity: A_ver_min (medium, Pd_DAS 0.969) ==\n');
s = T(T.kind == "A_ver_min" & T.scenario == "medium" & T.Pd_DAS == 0.969, :);   g = T(T.kind == "grid" & T.interdiction_case == "real22" & T.scenario == "medium" & T.Pd_DAS == 0.969, :);
s = [s; g];   [~, o] = sort(s.A_ver_min);   disp(s(o, {'A_ver_min','Hot_verified','fewest_drones_all_Hot_verified','drones_used','drone_stations'}));
fprintf('\n== Sensitivity: C_fiber (medium, Pd_DAS 0.969) ==\n');
s = T(T.kind == "C_fiber" | (T.kind == "grid" & T.interdiction_case == "real22" & T.scenario == "medium" & T.Pd_DAS == 0.969), :);   [~, o] = sort(s.C_fiber_factor);
disp(s(o, {'C_fiber_factor','C_total_SAR','fibre_share','interrogators','DAS_cells'}));
fprintf('\n== Pc_DAS 0.993 vs 0.983 (system accuracy, extra result) ==\n');
s = T(T.kind == "Pc_DAS 0.993", :);   disp(s(:, {'scenario','Pc_DAS','cells_system_accuracy_ok','min_system_accuracy'}));
fprintf('Pc_DAS 0.983 (grid, Pd 0.969): min system accuracy %s\n', mat2str(base.min_system_accuracy(base.Pd_DAS == 0.969)', 4));
end
