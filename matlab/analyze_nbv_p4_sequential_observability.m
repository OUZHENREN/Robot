function [steps, summary] = analyze_nbv_p4_sequential_observability(batchRoots, outputDir)
%ANALYZE_NBV_P4_SEQUENTIAL_OBSERVABILITY Evaluate preregistered P4 diagnostics.
%   This function is restricted to synthetic_view_dependent virtual data. It
%   does not establish real-camera accuracy, CAD ADD/ADD-S, or hardware NBV
%   performance. The success criterion is frozen in
%   docs/experiments/P4_SEQUENTIAL_OBSERVABILITY_PREREGISTRATION.md.

arguments
    batchRoots (1,:) string
    outputDir (1,1) string
end

for rootIndex = 1:numel(batchRoots)
    if ~isfolder(batchRoots(rootIndex))
        error('Batch root does not exist: %s', batchRoots(rootIndex));
    end
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

files = [];
for rootIndex = 1:numel(batchRoots)
    files = [files; dir(fullfile(batchRoots(rootIndex), 'episodes', '**', 'episode.csv'))]; %#ok<AGROW>
end
if isempty(files)
    error('No episode.csv files found below the supplied batch roots.');
end

runId = strings(0,1); strategy = strings(0,1); scene = strings(0,1);
seed = zeros(0,1); fromStep = zeros(0,1); toStep = zeros(0,1);
predictedReduction = zeros(0,1); observedReduction = zeros(0,1);
priorStd = zeros(0,1); predictedPosteriorStd = zeros(0,1);
posteriorStd = zeros(0,1); novelty = zeros(0,1); observability = zeros(0,1);

required = {'run_id','strategy_name','scene_name','random_seed','step', ...
    'trans_error','uncertainty_proxy_m','prior_covariance_translation_std_m', ...
    'predicted_posterior_covariance_translation_std_m', ...
    'covariance_translation_std_m','view_novelty','observability_score'};
for k = 1:numel(files)
    inputFile = fullfile(files(k).folder, files(k).name);
    episode = readtable(inputFile, 'TextType', 'string');
    missing = setdiff(required, episode.Properties.VariableNames);
    if ~isempty(missing)
        error('P4 field missing in %s: %s', inputFile, strjoin(missing, ', '));
    end
    episode = sortrows(episode, 'step');
    if height(episode) < 2 || episode.strategy_name(1) == "single_view"
        continue;
    end
    for row = 2:height(episode)
        runId(end+1,1) = string(episode.run_id(row)); %#ok<AGROW>
        strategy(end+1,1) = string(episode.strategy_name(row)); %#ok<AGROW>
        scene(end+1,1) = string(episode.scene_name(row)); %#ok<AGROW>
        seed(end+1,1) = episode.random_seed(row); %#ok<AGROW>
        fromStep(end+1,1) = episode.step(row-1); %#ok<AGROW>
        toStep(end+1,1) = episode.step(row); %#ok<AGROW>
        predictedReduction(end+1,1) = episode.uncertainty_proxy_m(row); %#ok<AGROW>
        observedReduction(end+1,1) = episode.trans_error(row-1) - episode.trans_error(row); %#ok<AGROW>
        priorStd(end+1,1) = episode.prior_covariance_translation_std_m(row); %#ok<AGROW>
        predictedPosteriorStd(end+1,1) = ...
            episode.predicted_posterior_covariance_translation_std_m(row); %#ok<AGROW>
        posteriorStd(end+1,1) = episode.covariance_translation_std_m(row); %#ok<AGROW>
        novelty(end+1,1) = episode.view_novelty(row); %#ok<AGROW>
        observability(end+1,1) = episode.observability_score(row); %#ok<AGROW>
    end
end
if isempty(predictedReduction)
    error('No multi-view P4 transitions were available for validation.');
end

occlusionCondition = localOcclusionCondition(scene);
steps = table(runId, strategy, scene, seed, fromStep, toStep, ...
    predictedReduction, observedReduction, priorStd, predictedPosteriorStd, ...
    posteriorStd, novelty, observability, occlusionCondition, 'VariableNames', ...
    {'run_id','strategy_name','scene_name','random_seed','from_step','to_step', ...
    'predicted_uncertainty_reduction_m','observed_translation_error_reduction_m', ...
    'prior_covariance_translation_std_m', ...
    'predicted_posterior_covariance_translation_std_m', ...
    'posterior_covariance_translation_std_m','view_novelty','observability_score', ...
    'occlusion_condition'});
writetable(steps, fullfile(outputDir, 'p4_sequential_observability_steps.csv'));

requiredConditions = ["none"; "light"; "heavy"];
requiredStrategies = ["uncertainty_only"; "path_cost_only"; "pose_gain"];
groupType = ["ALL"; repmat("OCCLUSION", numel(requiredConditions), 1)];
groupName = ["ALL"; requiredConditions];
n = zeros(numel(groupName),1); r = nan(numel(groupName),1);
ciLow = nan(numel(groupName),1); ciHigh = nan(numel(groupName),1);
meanPredicted = nan(numel(groupName),1); meanObserved = nan(numel(groupName),1);
for g = 1:numel(groupName)
    if groupType(g) == "ALL"
        mask = true(height(steps), 1);
    else
        mask = steps.occlusion_condition == groupName(g);
    end
    x = steps.predicted_uncertainty_reduction_m(mask);
    y = steps.observed_translation_error_reduction_m(mask);
    n(g) = numel(x);
    r(g) = localCorrelation(x, y);
    [ciLow(g), ciHigh(g)] = localBootstrapCI(x, y, 1000, 625 + g);
    meanPredicted(g) = mean(x, 'omitnan');
    meanObserved(g) = mean(y, 'omitnan');
end

designComplete = true;
for conditionIndex = 1:numel(requiredConditions)
    for strategyIndex = 1:numel(requiredStrategies)
        designMask = steps.occlusion_condition == requiredConditions(conditionIndex) & ...
            steps.strategy_name == requiredStrategies(strategyIndex);
        if numel(unique(steps.random_seed(designMask))) < 10
            designComplete = false;
        end
    end
end
overallPass = r(1) >= 0.30 && ciLow(1) > 0;
sceneDirectionPass = all(r(2:end) >= 0);
pass = designComplete && overallPass && sceneDirectionPass;
meetsPreregisteredDesign = repmat(designComplete, numel(groupName), 1);
summary = table(groupType, groupName, n, r, ciLow, ciHigh, ...
    meetsPreregisteredDesign, ...
    meanPredicted, meanObserved, 'VariableNames', {'group_type','group','n_steps', ...
    'pearson_r_predicted_vs_observed_reduction','bootstrap_ci95_low', ...
    'bootstrap_ci95_high','meets_preregistered_sample_design', ...
    'mean_predicted_reduction_m', ...
    'mean_observed_reduction_m'});
writetable(summary, fullfile(outputDir, 'p4_sequential_observability_summary.csv'));

figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 880 600]);
gscatter(steps.predicted_uncertainty_reduction_m, ...
    steps.observed_translation_error_reduction_m, steps.scene_name);
xlabel('Predicted translation-std reduction (m)');
ylabel('Observed next-step virtual translation-error reduction (m)');
title(sprintf('P4 virtual sequential-observability validation (n = %d)', height(steps)));
grid on; legend('Location', 'bestoutside', 'Interpreter', 'none');
exportgraphics(gcf, fullfile(outputDir, 'p4_prediction_vs_virtual_error_reduction.png'), ...
    'Resolution', 180);
close(gcf);

fid = fopen(fullfile(outputDir, 'p4_sequential_observability_summary.txt'), 'w');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'P4 virtual-only sequential-observability validation\n');
fprintf(fid, 'Batch roots: %s\n', strjoin(batchRoots, '; '));
fprintf(fid, 'Preregistered pass: r >= 0.30, 95%% bootstrap CI lower > 0, all scenes r >= 0\n');
fprintf(fid, 'Required design: none/light/heavy x 3 strategies x >=10 matched seeds per cell\n');
fprintf(fid, 'Dataset meets preregistered design: %s\n', ternary(designComplete, 'YES', 'NO'));
fprintf(fid, 'Decision: %s\n\n', ternary(pass, 'PASS', 'FAIL'));
for g = 1:height(summary)
    fprintf(fid, '%s/%s: n=%d, r=%.4f, CI=[%.4f, %.4f]\n', ...
        summary.group_type(g), summary.group(g), summary.n_steps(g), ...
        summary.pearson_r_predicted_vs_observed_reduction(g), ...
        summary.bootstrap_ci95_low(g), summary.bootstrap_ci95_high(g));
end
fprintf(fid, '\nCaveat: synthetic_view_dependent virtual data only; no real camera, robot, CAD ADD/ADD-S, or causal performance claim.\n');
end

function condition = localOcclusionCondition(sceneNames)
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

function [low, high] = localBootstrapCI(x, y, iterations, seed)
valid = isfinite(x) & isfinite(y);
x = x(valid); y = y(valid);
if numel(x) < 3
    low = NaN; high = NaN; return;
end
rng(seed, 'twister');
values = nan(iterations, 1);
for b = 1:iterations
    indices = randi(numel(x), numel(x), 1);
    values(b) = localCorrelation(x(indices), y(indices));
end
values = values(isfinite(values));
if isempty(values)
    low = NaN; high = NaN;
else
    bounds = prctile(values, [2.5, 97.5]);
    low = bounds(1); high = bounds(2);
end
end

function value = ternary(condition, trueValue, falseValue)
if condition
    value = trueValue;
else
    value = falseValue;
end
end
