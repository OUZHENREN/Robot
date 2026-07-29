function [steps, summary] = analyze_nbv_p5_calibration(batchRoots, outputDir)
%ANALYZE_NBV_P5_CALIBRATION Apply frozen P5 virtual-only calibration gates.
%   P5 passes only if the pose-aware visibility predictor agrees with the
%   synthetic sensor and the covariance floor prevents confidence collapse.

arguments
    batchRoots (1,:) string
    outputDir (1,1) string
end
for i = 1:numel(batchRoots)
    if ~isfolder(batchRoots(i)), error('Batch root does not exist: %s', batchRoots(i)); end
end
if ~isfolder(outputDir), mkdir(outputDir); end

files = [];
for i = 1:numel(batchRoots)
    files = [files; dir(fullfile(batchRoots(i), 'episodes', '**', 'episode.csv'))]; %#ok<AGROW>
    % A single smoke export may contain one episode directory directly,
    % whereas formal batches place episodes below an episodes/ directory.
    if isempty(files)
        files = [files; dir(fullfile(batchRoots(i), '**', 'episode.csv'))]; %#ok<AGROW>
    end
end
if isempty(files), error('No episode.csv files found.'); end

runId = strings(0,1); scene = strings(0,1); strategy = strings(0,1); seed = zeros(0,1);
step = zeros(0,1); predictedVisibility = zeros(0,1); actualVisibility = zeros(0,1);
posteriorStd = zeros(0,1); predictedReduction = zeros(0,1); observedReduction = zeros(0,1);
required = {'run_id','scene_name','strategy_name','random_seed','step','visible_ratio', ...
    'view_novelty','observability_score','covariance_translation_std_m', ...
    'uncertainty_proxy_m','trans_error'};
for k = 1:numel(files)
    episode = readtable(fullfile(files(k).folder, files(k).name), 'TextType', 'string');
    missing = setdiff(required, episode.Properties.VariableNames);
    if ~isempty(missing), error('P5 field missing: %s', strjoin(missing, ', ')); end
    episode = sortrows(episode, 'step');
    for row = 2:height(episode)
        runId(end+1,1) = episode.run_id(row); %#ok<AGROW>
        scene(end+1,1) = episode.scene_name(row); %#ok<AGROW>
        strategy(end+1,1) = episode.strategy_name(row); %#ok<AGROW>
        seed(end+1,1) = episode.random_seed(row); %#ok<AGROW>
        step(end+1,1) = episode.step(row); %#ok<AGROW>
        predictedVisibility(end+1,1) = episode.observability_score(row) / max(0.05, episode.view_novelty(row)); %#ok<AGROW>
        actualVisibility(end+1,1) = episode.visible_ratio(row); %#ok<AGROW>
        posteriorStd(end+1,1) = episode.covariance_translation_std_m(row); %#ok<AGROW>
        predictedReduction(end+1,1) = episode.uncertainty_proxy_m(row); %#ok<AGROW>
        observedReduction(end+1,1) = episode.trans_error(row-1) - episode.trans_error(row); %#ok<AGROW>
    end
end
occlusion = localOcclusion(scene);
steps = table(runId, scene, strategy, seed, step, predictedVisibility, actualVisibility, ...
    posteriorStd, predictedReduction, observedReduction, occlusion);
writetable(steps, fullfile(outputDir, 'p5_calibration_steps.csv'));

groups = ["ALL"; "none"; "light"; "heavy"];
n = zeros(4,1); r = nan(4,1); mae = nan(4,1); collapse = nan(4,1); exploratoryR = nan(4,1);
for g = 1:numel(groups)
    mask = true(height(steps),1);
    if groups(g) ~= "ALL", mask = steps.occlusion == groups(g); end
    data = steps(mask,:); n(g) = height(data);
    r(g) = localCorrelation(data.predictedVisibility, data.actualVisibility);
    mae(g) = mean(abs(data.predictedVisibility - data.actualVisibility), 'omitnan');
    collapse(g) = mean(data.posteriorStd < 1e-4);
    exploratoryR(g) = localCorrelation(data.predictedReduction, data.observedReduction);
end

conditions = ["none"; "light"; "heavy"];
designComplete = true;
for c = 1:numel(conditions)
    mask = steps.occlusion == conditions(c) & steps.strategy == "fixed_order";
    if numel(unique(steps.seed(mask))) < 10, designComplete = false; end
end
calibrationPass = designComplete && r(1) >= 0.70 && all(mae(2:end) <= 0.08) && all(collapse(2:end) <= 0.10);
summary = table(groups, n, r, mae, collapse, exploratoryR, repmat(designComplete,4,1), ...
    'VariableNames', {'group','n_steps','pearson_r_predicted_vs_actual_visibility', ...
    'mean_absolute_visibility_error','posterior_std_below_0p1mm_fraction', ...
    'exploratory_r_predicted_reduction_vs_error_reduction','meets_preregistered_sample_design'});
writetable(summary, fullfile(outputDir, 'p5_calibration_summary.csv'));

figure('Visible','off','Color','w','Position',[100 100 800 560]);
gscatter(steps.predictedVisibility, steps.actualVisibility, steps.occlusion);
xlabel('Predicted z-buffer visible fraction'); ylabel('Actual synthetic visible fraction');
title(sprintf('P5 visibility calibration (n = %d)', height(steps))); grid on;
legend('Location','bestoutside');
exportgraphics(gcf, fullfile(outputDir, 'p5_visibility_calibration.png'), 'Resolution', 180); close(gcf);

fid = fopen(fullfile(outputDir, 'p5_calibration_summary.txt'), 'w'); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'P5 pose-aware virtual-only calibration\n');
fprintf(fid, 'Required design: none/light/heavy x fixed_order x >=10 matched seeds\n');
fprintf(fid, 'Pass: all design cells present; ALL visibility r >= 0.70; each-scene MAE <= 0.08; each-scene collapse <= 10%%\n');
fprintf(fid, 'Dataset meets preregistered design: %s\n', ternary(designComplete, 'YES', 'NO'));
fprintf(fid, 'Decision: %s\n\n', ternary(calibrationPass, 'PASS', 'FAIL'));
for g = 1:height(summary)
    fprintf(fid, '%s: n=%d, visibility r=%.4f, MAE=%.4f, collapse=%.1f%%, exploratory proxy->error r=%.4f\n', ...
        summary.group(g), summary.n_steps(g), summary.pearson_r_predicted_vs_actual_visibility(g), ...
        summary.mean_absolute_visibility_error(g), 100*summary.posterior_std_below_0p1mm_fraction(g), ...
        summary.exploratory_r_predicted_reduction_vs_error_reduction(g));
end
fprintf(fid, '\nVirtual-only calibration; no real-camera, robot, CAD ADD/ADD-S, or strategy-superiority claim.\n');
end

function condition = localOcclusion(names)
condition = repmat("unclassified", numel(names), 1); names = lower(string(names));
condition(contains(names,"none")) = "none"; condition(contains(names,"light")) = "light"; condition(contains(names,"heavy")) = "heavy";
end
function r = localCorrelation(x,y)
valid = isfinite(x) & isfinite(y); x=x(valid); y=y(valid);
if numel(x)<3 || std(x)==0 || std(y)==0, r=NaN; else, c=corrcoef(x,y); r=c(1,2); end
end
function result = ternary(condition, yes, no)
if condition, result=yes; else, result=no; end
end
