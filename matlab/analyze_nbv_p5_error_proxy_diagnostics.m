function [transitions, summary] = analyze_nbv_p5_error_proxy_diagnostics(batchRoots, outputDir)
%ANALYZE_NBV_P5_ERROR_PROXY_DIAGNOSTICS Diagnose, rather than over-interpret,
% the P5 predicted-uncertainty versus truth-error comparison.
%
% `uncertainty_proxy_m` is a predicted translation-standard-deviation
% reduction.  It is not a predicted reduction in ground-truth ICP error.
% This program therefore reports the aligned covariance endpoint separately
% from the exploratory truth-error endpoint, and always stratifies first
% views from later views.  It makes no strategy-efficacy decision.

arguments
    batchRoots (1,:) string
    outputDir (1,1) string
end
if ~isfolder(outputDir), mkdir(outputDir); end
files = [];
for root = batchRoots
    if ~isfolder(root), error('Batch root does not exist: %s', root); end
    files = [files; dir(fullfile(root, 'episodes', '**', 'episode.csv'))]; %#ok<AGROW>
end
if isempty(files), error('No episode.csv files found below episodes/.'); end

runId = strings(0,1); occlusion = strings(0,1); step = zeros(0,1);
expectedStdReduction = zeros(0,1); observedStdReduction = zeros(0,1);
priorError = zeros(0,1); postError = zeros(0,1); errorReduction = zeros(0,1);
visibility = zeros(0,1); initialBias = zeros(0,1);
required = {'run_id','scene_name','step','trans_error','uncertainty_proxy_m', ...
    'covariance_translation_std_m','prior_covariance_translation_std_m', ...
    'predicted_posterior_covariance_translation_std_m','visible_ratio'};

for k = 1:numel(files)
    episode = readtable(fullfile(files(k).folder, files(k).name), 'TextType', 'string');
    missing = setdiff(required, episode.Properties.VariableNames);
    if ~isempty(missing), error('Required P5 field missing: %s', strjoin(missing, ', ')); end
    episode = sortrows(episode, 'step');
    for row = 2:height(episode)
        runId(end+1,1) = episode.run_id(row); %#ok<AGROW>
        occlusion(end+1,1) = localOcclusion(episode.scene_name(row)); %#ok<AGROW>
        step(end+1,1) = episode.step(row); %#ok<AGROW>
        expectedStdReduction(end+1,1) = episode.uncertainty_proxy_m(row); %#ok<AGROW>
        if ismember('observed_covariance_translation_std_reduction_m', episode.Properties.VariableNames)
            observedStdReduction(end+1,1) = ...
                episode.observed_covariance_translation_std_reduction_m(row); %#ok<AGROW>
        else
            observedStdReduction(end+1,1) = ...
                episode.covariance_translation_std_m(row-1) - ...
                episode.covariance_translation_std_m(row); %#ok<AGROW>
        end
        priorError(end+1,1) = episode.trans_error(row-1); %#ok<AGROW>
        postError(end+1,1) = episode.trans_error(row); %#ok<AGROW>
        errorReduction(end+1,1) = priorError(end) - postError(end); %#ok<AGROW>
        visibility(end+1,1) = episode.visible_ratio(row); %#ok<AGROW>
        if ismember('virtual_initial_translation_bias_m', episode.Properties.VariableNames)
            initialBias(end+1,1) = episode.virtual_initial_translation_bias_m(row); %#ok<AGROW>
        else
            initialBias(end+1,1) = 0; %#ok<AGROW>
        end
    end
end

transitions = table(runId, occlusion, step, expectedStdReduction, observedStdReduction, ...
    priorError, postError, errorReduction, visibility, initialBias);
writetable(transitions, fullfile(outputDir, 'p5_error_proxy_transitions.csv'));

groupNames = ["ALL"; "first_view"; "later_view"; "none"; "light"; "heavy"];
summary = table('Size',[numel(groupNames),8], ...
    'VariableTypes', {'string','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'group','n_transitions','r_expected_vs_observed_covariance_reduction', ...
    'r_expected_vs_truth_error_reduction','r_prior_error_vs_truth_error_reduction', ...
    'r_expected_vs_visibility','mean_expected_std_reduction_m', ...
    'fraction_truth_error_worsened'});
for g = 1:numel(groupNames)
    mask = localMask(transitions, groupNames(g));
    x = transitions(mask,:);
    summary.group(g) = groupNames(g);
    summary.n_transitions(g) = height(x);
    summary.r_expected_vs_observed_covariance_reduction(g) = ...
        localCorr(x.expectedStdReduction, x.observedStdReduction);
    summary.r_expected_vs_truth_error_reduction(g) = ...
        localCorr(x.expectedStdReduction, x.errorReduction);
    summary.r_prior_error_vs_truth_error_reduction(g) = ...
        localCorr(x.priorError, x.errorReduction);
    summary.r_expected_vs_visibility(g) = localCorr(x.expectedStdReduction, x.visibility);
    summary.mean_expected_std_reduction_m(g) = mean(x.expectedStdReduction, 'omitnan');
    summary.fraction_truth_error_worsened(g) = mean(x.errorReduction < 0, 'omitnan');
end
writetable(summary, fullfile(outputDir, 'p5_error_proxy_diagnostics_summary.csv'));

figure('Visible','off','Color','w','Position',[100 100 1040 440]);
tiledlayout(1,2, 'TileSpacing','compact');
nexttile; gscatter(transitions.expectedStdReduction, transitions.observedStdReduction, transitions.occlusion);
xlabel('Predicted translation std reduction (m)'); ylabel('Observed covariance std reduction (m)');
title('Aligned covariance endpoint'); grid on;
nexttile; gscatter(transitions.expectedStdReduction, transitions.errorReduction, transitions.occlusion);
xlabel('Predicted translation std reduction (m)'); ylabel('Next-step truth-error reduction (m)');
title('Exploratory, non-aligned error endpoint'); grid on;
exportgraphics(gcf, fullfile(outputDir, 'p5_error_proxy_diagnostics.png'), 'Resolution', 180); close(gcf);

fid = fopen(fullfile(outputDir, 'p5_error_proxy_diagnostics.txt'), 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'P5 predicted-uncertainty / error-proxy diagnostic\n');
fprintf(fid, 'The predictor targets covariance standard-deviation reduction, not immediate ICP truth-error reduction.\n');
fprintf(fid, 'No strategy-efficacy claim is permitted from this diagnostic.\n\n');
for g = 1:height(summary)
    fprintf(fid, '%s: n=%d; r(expected, observed covariance reduction)=%.4f; ', ...
        summary.group(g), summary.n_transitions(g), ...
        summary.r_expected_vs_observed_covariance_reduction(g));
    fprintf(fid, 'r(expected, truth-error reduction)=%.4f; r(prior error, truth-error reduction)=%.4f; worsened=%.1f%%\n', ...
        summary.r_expected_vs_truth_error_reduction(g), ...
        summary.r_prior_error_vs_truth_error_reduction(g), ...
        100 * summary.fraction_truth_error_worsened(g));
end
fprintf(fid, '\nP6 remedy: use a declared virtual-only initial pose bias and paired covariance, then compare\n');
fprintf(fid, 'matched strategies on episode-level final truth error under independent deterministic sensor seeds.\n');
end

function mask = localMask(data, group)
switch group
    case "ALL", mask = true(height(data),1);
    case "first_view", mask = data.step == 1;
    case "later_view", mask = data.step > 1;
    otherwise, mask = data.occlusion == group;
end
end

function value = localCorr(x, y)
valid = isfinite(x) & isfinite(y); x = x(valid); y = y(valid);
if numel(x) < 3 || std(x) == 0 || std(y) == 0
    value = NaN;
else
    c = corrcoef(x, y); value = c(1,2);
end
end

function condition = localOcclusion(scene)
scene = lower(string(scene));
if contains(scene, 'none'), condition = "none";
elseif contains(scene, 'light'), condition = "light";
elseif contains(scene, 'heavy'), condition = "heavy";
else, condition = "unclassified";
end
end
