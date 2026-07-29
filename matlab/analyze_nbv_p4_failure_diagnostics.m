function [transitions, summary] = analyze_nbv_p4_failure_diagnostics(batchRoots, outputDir)
%ANALYZE_NBV_P4_FAILURE_DIAGNOSTICS Audit why the frozen P4 proxy failed.
%   Reads P4 synthetic_view_dependent episode CSVs without modifying them.
%   The report quantifies covariance collapse, proxy dynamic range and the
%   mismatch between P4's projected-visibility proxy and realised coverage.
%   It is diagnostic only: it must not be used to reinterpret P4 as a pass.

arguments
    batchRoots (1,:) string
    outputDir (1,1) string
end

for i = 1:numel(batchRoots)
    if ~isfolder(batchRoots(i))
        error('Batch root does not exist: %s', batchRoots(i));
    end
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

files = [];
for i = 1:numel(batchRoots)
    files = [files; dir(fullfile(batchRoots(i), 'episodes', '**', 'episode.csv'))]; %#ok<AGROW>
end
if isempty(files)
    error('No episode.csv files found below the supplied batch roots.');
end

runId = strings(0,1); scene = strings(0,1); strategy = strings(0,1);
seed = zeros(0,1); fromStep = zeros(0,1); toStep = zeros(0,1);
predictedReduction = zeros(0,1); observedReduction = zeros(0,1);
predictedVisibility = zeros(0,1); actualVisibility = zeros(0,1);
priorStd = zeros(0,1); predictedPosteriorStd = zeros(0,1);
posteriorStd = zeros(0,1); currentError = zeros(0,1); rmse = zeros(0,1);

required = {'run_id','scene_name','strategy_name','random_seed','step', ...
    'trans_error','registration_rmse_m','uncertainty_proxy_m', ...
    'prior_covariance_translation_std_m', ...
    'predicted_posterior_covariance_translation_std_m', ...
    'covariance_translation_std_m','visible_ratio','view_novelty', ...
    'observability_score'};

for k = 1:numel(files)
    inputFile = fullfile(files(k).folder, files(k).name);
    episode = readtable(inputFile, 'TextType', 'string');
    missing = setdiff(required, episode.Properties.VariableNames);
    if ~isempty(missing)
        error('P4 field missing in %s: %s', inputFile, strjoin(missing, ', '));
    end
    episode = sortrows(episode, 'step');
    for row = 2:height(episode)
        runId(end+1,1) = episode.run_id(row); %#ok<AGROW>
        scene(end+1,1) = episode.scene_name(row); %#ok<AGROW>
        strategy(end+1,1) = episode.strategy_name(row); %#ok<AGROW>
        seed(end+1,1) = episode.random_seed(row); %#ok<AGROW>
        fromStep(end+1,1) = episode.step(row-1); %#ok<AGROW>
        toStep(end+1,1) = episode.step(row); %#ok<AGROW>
        predictedReduction(end+1,1) = episode.uncertainty_proxy_m(row); %#ok<AGROW>
        observedReduction(end+1,1) = episode.trans_error(row-1) - episode.trans_error(row); %#ok<AGROW>
        % P4 defines observability = projected_visibility * novelty. The
        % quotient recovers its pre-observation projected visibility proxy.
        predictedVisibility(end+1,1) = episode.observability_score(row) / ...
            max(0.05, episode.view_novelty(row)); %#ok<AGROW>
        actualVisibility(end+1,1) = episode.visible_ratio(row); %#ok<AGROW>
        priorStd(end+1,1) = episode.prior_covariance_translation_std_m(row); %#ok<AGROW>
        predictedPosteriorStd(end+1,1) = ...
            episode.predicted_posterior_covariance_translation_std_m(row); %#ok<AGROW>
        posteriorStd(end+1,1) = episode.covariance_translation_std_m(row); %#ok<AGROW>
        currentError(end+1,1) = episode.trans_error(row); %#ok<AGROW>
        rmse(end+1,1) = episode.registration_rmse_m(row); %#ok<AGROW>
    end
end

if isempty(runId)
    error('No multi-view transitions were available for P4 diagnostics.');
end

occlusion = localOcclusion(scene);
transitions = table(runId, scene, strategy, seed, fromStep, toStep, ...
    predictedReduction, observedReduction, predictedVisibility, actualVisibility, ...
    priorStd, predictedPosteriorStd, posteriorStd, currentError, rmse, occlusion);
writetable(transitions, fullfile(outputDir, 'p4_failure_diagnostic_transitions.csv'));

groups = ["ALL"; "none"; "light"; "heavy"];
n = zeros(numel(groups),1); collapseFraction = nan(numel(groups),1);
proxyR = nan(numel(groups),1); coverageR = nan(numel(groups),1);
posteriorErrorR = nan(numel(groups),1); visibilityR = nan(numel(groups),1);
proxyQ05 = nan(numel(groups),1); proxyMedian = nan(numel(groups),1); proxyQ95 = nan(numel(groups),1);
actualVisibilityMean = nan(numel(groups),1); predictedVisibilityMean = nan(numel(groups),1);

for g = 1:numel(groups)
    if groups(g) == "ALL"
        mask = true(height(transitions), 1);
    else
        mask = transitions.occlusion == groups(g);
    end
    data = transitions(mask, :);
    n(g) = height(data);
    collapseFraction(g) = mean(data.posteriorStd < 1e-4);
    proxyR(g) = localCorrelation(data.predictedReduction, data.observedReduction);
    coverageR(g) = localCorrelation(data.actualVisibility, data.observedReduction);
    posteriorErrorR(g) = localCorrelation(data.posteriorStd, data.currentError);
    visibilityR(g) = localCorrelation(data.predictedVisibility, data.actualVisibility);
    quantiles = prctile(data.predictedReduction, [5, 50, 95]);
    proxyQ05(g) = quantiles(1); proxyMedian(g) = quantiles(2); proxyQ95(g) = quantiles(3);
    actualVisibilityMean(g) = mean(data.actualVisibility, 'omitnan');
    predictedVisibilityMean(g) = mean(data.predictedVisibility, 'omitnan');
end

summary = table(groups, n, collapseFraction, proxyR, coverageR, posteriorErrorR, ...
    visibilityR, proxyQ05, proxyMedian, proxyQ95, actualVisibilityMean, ...
    predictedVisibilityMean, 'VariableNames', {'group','n_transitions', ...
    'posterior_std_below_0p1mm_fraction', ...
    'proxy_vs_error_reduction_r','actual_coverage_vs_error_reduction_r', ...
    'posterior_std_vs_current_error_r','predicted_vs_actual_visibility_r', ...
    'predicted_reduction_q05_m','predicted_reduction_median_m', ...
    'predicted_reduction_q95_m','actual_visibility_mean', ...
    'predicted_visibility_mean'});
writetable(summary, fullfile(outputDir, 'p4_failure_diagnostic_summary.csv'));

figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 380]);
tiledlayout(1,2, 'Padding', 'compact');
nexttile;
gscatter(transitions.predictedVisibility, transitions.actualVisibility, transitions.occlusion);
xlabel('P4 projected-visibility proxy'); ylabel('Realised virtual visible ratio');
title('P4 prediction versus virtual sensor coverage'); grid on;
legend('Location', 'bestoutside');
nexttile;
gscatter(transitions.toStep, transitions.posteriorStd * 1e3, transitions.occlusion);
xlabel('View step'); ylabel('Fused translation std (mm)');
title('P4 sequential covariance collapse'); grid on;
exportgraphics(gcf, fullfile(outputDir, 'p4_failure_diagnostic.png'), 'Resolution', 180);
close(gcf);

fid = fopen(fullfile(outputDir, 'p4_failure_diagnostic_summary.txt'), 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'P4 immutable-data failure diagnostic (virtual-only)\n');
fprintf(fid, 'Input batch roots: %s\n', strjoin(batchRoots, '; '));
fprintf(fid, 'This report does not alter P4 inputs or its preregistered decision.\n\n');
for g = 1:height(summary)
    fprintf(fid, ['%s: n=%d; collapse(<0.1mm)=%.1f%%; proxy->error r=%.4f; ' ...
        'actual coverage->error r=%.4f; predicted->actual visibility r=%.4f; ' ...
        'proxy q05/median/q95=[%.3g, %.3g, %.3g] m\n'], ...
        summary.group(g), summary.n_transitions(g), ...
        100 * summary.posterior_std_below_0p1mm_fraction(g), ...
        summary.proxy_vs_error_reduction_r(g), ...
        summary.actual_coverage_vs_error_reduction_r(g), ...
        summary.predicted_vs_actual_visibility_r(g), ...
        summary.predicted_reduction_q05_m(g), ...
        summary.predicted_reduction_median_m(g), ...
        summary.predicted_reduction_q95_m(g));
end
fprintf(fid, ['\nInterpretation: a high collapse fraction or near-zero proxy range indicates ' ...
    'overconfident information fusion. A weak predicted-versus-realised visibility ' ...
    'correlation indicates that the predictor and synthetic sensor do not share a ' ...
    'measurement model. These are failure diagnoses, not post-hoc success criteria.\n']);
end

function condition = localOcclusion(sceneNames)
condition = repmat("unclassified", numel(sceneNames), 1);
names = lower(string(sceneNames));
condition(contains(names, "none")) = "none";
condition(contains(names, "light")) = "light";
condition(contains(names, "heavy")) = "heavy";
end

function r = localCorrelation(x, y)
valid = isfinite(x) & isfinite(y);
x = x(valid); y = y(valid);
if numel(x) < 3 || std(x) == 0 || std(y) == 0
    r = NaN;
else
    c = corrcoef(x, y);
    r = c(1,2);
end
end
