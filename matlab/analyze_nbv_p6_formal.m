function [episodes, pairs, summary] = analyze_nbv_p6_formal(batchRoots, outputDir)
%ANALYZE_NBV_P6_FORMAL Apply the frozen P6 matched-seed decision rule.
%
% P6 is virtual-only.  It tests final truth translation error after a declared
% virtual initial bias; it does not support a real-camera or hardware claim.
% The primary quantity is baseline final error minus pose_gain final error, so
% a positive value favours pose_gain.

arguments
    batchRoots (1,:) string
    outputDir (1,1) string
end
if ~isfolder(outputDir), mkdir(outputDir); end

files = [];
for root = batchRoots
    if ~isfolder(root), error('Batch root does not exist: %s', root); end
    files = [files; dir(fullfile(root, 'episodes', '**', 'report_summary.csv'))]; %#ok<AGROW>
end
if isempty(files), error('No P6 report_summary.csv files found.'); end

runId = strings(0,1); scene = strings(0,1); strategy = strings(0,1); seed = zeros(0,1);
finalError = zeros(0,1); converged = false(0,1); failure = strings(0,1);
required = {'run_id','data_source','validity_label','strategy_name','scene_name', ...
    'random_seed','final_translation_error_m','converged','failure_reason'};
for k = 1:numel(files)
    row = readtable(fullfile(files(k).folder, files(k).name), 'TextType', 'string');
    missing = setdiff(required, row.Properties.VariableNames);
    if ~isempty(missing), error('P6 field missing: %s', strjoin(missing, ', ')); end
    if height(row) ~= 1, error('Expected one summary row in %s', files(k).folder); end
    if row.data_source ~= "synthetic_view_dependent" || row.validity_label ~= "research_candidate"
        error('Non-P6 provenance in %s', files(k).folder);
    end
    configPath = fullfile(files(k).folder, 'config.yaml');
    if ~isfile(configPath), error('Missing P6 config snapshot: %s', files(k).folder); end
    configText = string(fileread(configPath));
    if ~contains(configText, ...
            "sensor_noise_seed_contract: random_seed_plus_7919_times_view_index_reset_per_episode")
        error('P6 sensor-noise seed contract missing in %s', configPath);
    end
    if ~contains(configText, "virtual_initial_translation_bias_m: 0.015") || ...
            ~contains(configText, "virtual_initial_covariance_std_m: 0.03")
        error('P6 controlled initial condition missing in %s', configPath);
    end
    runId(end+1,1) = row.run_id; %#ok<AGROW>
    scene(end+1,1) = localScene(row.scene_name); %#ok<AGROW>
    strategy(end+1,1) = row.strategy_name; %#ok<AGROW>
    seed(end+1,1) = row.random_seed; %#ok<AGROW>
    finalError(end+1,1) = row.final_translation_error_m; %#ok<AGROW>
    converged(end+1,1) = localLogical(row.converged); %#ok<AGROW>
    failure(end+1,1) = row.failure_reason; %#ok<AGROW>
end
episodes = table(runId, scene, strategy, seed, finalError, converged, failure);
writetable(episodes, fullfile(outputDir, 'p6_episode_summary.csv'));

scenes = ["none"; "light"; "heavy"];
strategies = ["fixed_order"; "random_reachable"; "pose_gain"];
designComplete = true;
for s = 1:numel(scenes)
    seedSets = cell(numel(strategies),1);
    for q = 1:numel(strategies)
        mask = episodes.scene == scenes(s) & episodes.strategy == strategies(q);
        seedSets{q} = unique(episodes.seed(mask));
        if numel(seedSets{q}) < 20 || sum(mask) ~= numel(seedSets{q})
            designComplete = false;
        end
    end
    if ~isequal(seedSets{1}, seedSets{2}, seedSets{3}), designComplete = false; end
end
if any(episodes.failure ~= "")
    designComplete = false;
end

pairScene = strings(0,1); pairSeed = zeros(0,1); comparison = strings(0,1);
baselineError = zeros(0,1); poseGainError = zeros(0,1); improvement = zeros(0,1);
comparators = ["fixed_order"; "random_reachable"];
for s = 1:numel(scenes)
    matchedSeeds = unique(episodes.seed(episodes.scene == scenes(s) & episodes.strategy == "pose_gain"));
    for seedValue = reshape(matchedSeeds, 1, [])
        poseRow = episodes(episodes.scene == scenes(s) & episodes.strategy == "pose_gain" & episodes.seed == seedValue,:);
        for c = 1:numel(comparators)
            baseRow = episodes(episodes.scene == scenes(s) & episodes.strategy == comparators(c) & episodes.seed == seedValue,:);
            if height(poseRow) ~= 1 || height(baseRow) ~= 1, continue; end
            pairScene(end+1,1) = scenes(s); %#ok<AGROW>
            pairSeed(end+1,1) = seedValue; %#ok<AGROW>
            comparison(end+1,1) = comparators(c) + "_minus_pose_gain"; %#ok<AGROW>
            baselineError(end+1,1) = baseRow.finalError; %#ok<AGROW>
            poseGainError(end+1,1) = poseRow.finalError; %#ok<AGROW>
            improvement(end+1,1) = baseRow.finalError - poseRow.finalError; %#ok<AGROW>
        end
    end
end
pairs = table(pairScene, pairSeed, comparison, baselineError, poseGainError, improvement);
writetable(pairs, fullfile(outputDir, 'p6_matched_pairs.csv'));

summary = table('Size',[8,7], ...
    'VariableTypes', {'string','string','double','double','double','double','double'}, ...
    'VariableNames', {'scope','comparison','n_pairs','median_improvement_m', ...
    'bootstrap_ci_low_m','bootstrap_ci_high_m','mean_improvement_m'});
row = 0;
for scope = ["pooled"; "heavy"]'
    for c = 1:numel(comparators)
        row = row + 1;
        cmp = comparators(c) + "_minus_pose_gain";
        mask = pairs.comparison == cmp;
        if scope == "heavy", mask = mask & pairs.pairScene == "heavy"; end
        values = pairs.improvement(mask);
        [ciLow, ciHigh] = localBootstrapMedianCI(values, 10000, 6252026 + row);
        summary.scope(row) = scope; summary.comparison(row) = cmp;
        summary.n_pairs(row) = numel(values); summary.median_improvement_m(row) = median(values, 'omitnan');
        summary.bootstrap_ci_low_m(row) = ciLow; summary.bootstrap_ci_high_m(row) = ciHigh;
        summary.mean_improvement_m(row) = mean(values, 'omitnan');
    end
end
summary = summary(1:row,:);
writetable(summary, fullfile(outputDir, 'p6_decision_summary.csv'));

pooled = summary(summary.scope == "pooled",:);
heavy = summary(summary.scope == "heavy",:);
supported = designComplete && all(pooled.bootstrap_ci_low_m > 0) && ...
    all(heavy.median_improvement_m >= 0);

figure('Visible','off','Color','w','Position',[100 100 850 520]);
groups = categorical(pairs.pairScene + " / " + pairs.comparison);
boxchart(groups, pairs.improvement); hold on; yline(0, '--k');
ylabel('Baseline final error - PoseGain final error (m)');
title('P6 matched-seed virtual-only final-error improvement'); grid on;
xtickangle(30); exportgraphics(gcf, fullfile(outputDir, 'p6_matched_improvement.png'), 'Resolution', 180); close(gcf);

fid = fopen(fullfile(outputDir, 'p6_decision.txt'), 'w'); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'P6 controlled virtual error-reduction decision\n');
fprintf(fid, 'Design required: 3 scenes x 3 strategies x >=20 identical seeds; no failed episodes.\n');
fprintf(fid, 'Decision rule: each pooled baseline-minus-PoseGain median bootstrap CI lower bound > 0; heavy medians do not reverse.\n');
fprintf(fid, 'Design complete: %s\n', ternary(designComplete, 'YES', 'NO'));
for i = 1:height(summary)
    fprintf(fid, '%s / %s: n=%d, median=%.6f m, 95%% CI [%.6f, %.6f] m, mean=%.6f m\n', ...
        summary.scope(i), summary.comparison(i), summary.n_pairs(i), ...
        summary.median_improvement_m(i), summary.bootstrap_ci_low_m(i), ...
        summary.bootstrap_ci_high_m(i), summary.mean_improvement_m(i));
end
fprintf(fid, 'Decision: %s\n', ternary(supported, 'SUPPORTED IN THIS VIRTUAL BENCHMARK', 'UNSUPPORTED'));
fprintf(fid, 'No real-camera, hardware, or CAD ADD/ADD-S claim follows from this output.\n');
end

function scene = localScene(value)
value = lower(string(value));
if contains(value, 'none'), scene = "none";
elseif contains(value, 'light'), scene = "light";
elseif contains(value, 'heavy'), scene = "heavy";
else, error('Unclassified scene: %s', value);
end
end

function value = localLogical(raw)
value = any(lower(string(raw)) == ["true","1"]);
end

function [low, high] = localBootstrapMedianCI(values, nBoot, seed)
values = values(isfinite(values));
if isempty(values), low = NaN; high = NaN; return; end
rng(seed, 'twister'); n = numel(values); medians = zeros(nBoot,1);
for b = 1:nBoot, medians(b) = median(values(randi(n, n, 1))); end
low = prctile(medians, 2.5); high = prctile(medians, 97.5);
end

function out = ternary(condition, yes, no)
if condition, out = yes; else, out = no; end
end
