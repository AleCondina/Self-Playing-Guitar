function [raw, segmentSummary, scoreSummary] = pitest_import(logFile, saveMatFile)
%PITEST_IMPORT Import and score UART logs from the playback PI test harness.
%
% Usage:
%   [raw, segmentSummary, scoreSummary] = pitest_import("pitest_log.txt");
%   [raw, segmentSummary, scoreSummary] = pitest_import("pitest_log.txt", "pitest_log.mat");
%
% The importer:
%   1. Strips the "[PIT] " prefix
%   2. Loads the per-hop CSV rows into a table
%   3. Builds per-segment metrics grouped by:
%        test_name, string_id, rep, segment_idx
%   4. Builds per-test aggregate scores grouped by:
%        test_name, string_id

if nargin < 2
    saveMatFile = "";
end

text = fileread(logFile);
lines = splitlines(string(text));
lines = strip(lines);

header = "timestamp_ms,test_name,rep,segment_idx,string_id,target_hz,measured_hz,error_hz,error_cents,ff_steps,trim_steps,cmd_steps,cur_steps,lock,pi_en,confidence,faults";
dataLines = strings(0, 1);

for i = 1:numel(lines)
    line = lines(i);
    if strlength(line) < 6
        continue;
    end

    pitIdx = strfind(char(line), "[PIT] ");
    if isempty(pitIdx)
        continue;
    end

    payload = extractAfter(line, pitIdx(1) + 5);
    if payload == header
        continue;
    end
    if startsWith(payload, "META,") || startsWith(payload, "DONE,")
        continue;
    end
    if isempty(payload) || ~isstrprop(extractBefore(payload, 2), "digit")
        continue;
    end

    dataLines(end + 1, 1) = payload; %#ok<AGROW>
end

if isempty(dataLines)
    fprintf('\n=== PITEST IMPORT ===\n');
    fprintf('File: %s\n', char(logFile));
    fprintf('No [PIT] data rows found.\n');
    raw = table();
    segmentSummary = table();
    scoreSummary = table();
    return;
end

csvBlob = strjoin([header; dataLines], newline);
formats = "%f%s%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f";
parsed = textscan(char(csvBlob), formats, 'Delimiter', ',', 'HeaderLines', 1, 'CollectOutput', false);

raw = table( ...
    parsed{1}, string(parsed{2}), parsed{3}, parsed{4}, parsed{5}, ...
    parsed{6}, parsed{7}, parsed{8}, parsed{9}, parsed{10}, parsed{11}, ...
    parsed{12}, parsed{13}, parsed{14}, parsed{15}, parsed{16}, parsed{17}, ...
    'VariableNames', { ...
        'timestamp_ms', 'test_name', 'rep', 'segment_idx', 'string_id', ...
        'target_hz', 'measured_hz', 'error_hz', 'error_cents', 'ff_steps', ...
        'trim_steps', 'cmd_steps', 'cur_steps', 'lock', 'pi_en', ...
        'confidence', 'faults'});

validMeas = raw.measured_hz > 0;
raw.error_hz(~validMeas) = NaN;
raw.error_cents(~validMeas) = NaN;
raw.abs_error_cents = abs(raw.error_cents);

[segGroups, segTestName, segString, segRep, segIdx] = findgroups( ...
    raw.test_name, raw.string_id, raw.rep, raw.segment_idx);

segmentSummary = table(segTestName, segString, segRep, segIdx, ...
    'VariableNames', {'test_name', 'string_id', 'rep', 'segment_idx'});

segmentSummary.lock_success = splitapply(@segmentLockSuccess, raw.lock, segGroups);
segmentSummary.lock_time_ms = splitapply(@segmentLockTime, raw.timestamp_ms, raw.lock, segGroups);
segmentSummary.max_abs_error_cents = splitapply(@segmentMaxAbsError, raw.abs_error_cents, segGroups);
segmentSummary.mean_abs_error_cents_last_1s = splitapply(@segmentMeanLast1s, raw.timestamp_ms, raw.abs_error_cents, segGroups);
segmentSummary.std_error_cents_last_1s = splitapply(@segmentStdLast1s, raw.timestamp_ms, raw.error_cents, segGroups);
segmentSummary.unlock_count_after_first_lock = splitapply(@segmentUnlockCount, raw.lock, segGroups);
segmentSummary.fault_count = splitapply(@segmentFaultCount, raw.faults, segGroups);
segmentSummary.trim_rms = splitapply(@segmentTrimRms, raw.trim_steps, segGroups);

[scoreGroups, scoreTestName, scoreString] = findgroups(segmentSummary.test_name, segmentSummary.string_id);
scoreSummary = table(scoreTestName, scoreString, ...
    'VariableNames', {'test_name', 'string_id'});

scoreSummary.lock_fail_rate = splitapply(@(x) mean(~x), segmentSummary.lock_success, scoreGroups);
scoreSummary.unlock_rate = splitapply(@(x) mean(x > 0), segmentSummary.unlock_count_after_first_lock, scoreGroups);
scoreSummary.median_lock_time_ms = splitapply(@medianNaN, segmentSummary.lock_time_ms, scoreGroups);
scoreSummary.P95_max_abs_error_cents = splitapply(@p95NaN, segmentSummary.max_abs_error_cents, scoreGroups);
scoreSummary.P95_mean_abs_error_cents_last_1s = splitapply(@p95NaN, segmentSummary.mean_abs_error_cents_last_1s, scoreGroups);
scoreSummary.fault_rate = splitapply(@(x) mean(x > 0), segmentSummary.fault_count, scoreGroups);

scoreSummary.Score = 100 .* scoreSummary.lock_fail_rate ...
                   + 30  .* scoreSummary.unlock_rate ...
                   + 0.02 .* scoreSummary.median_lock_time_ms ...
                   + 0.5 .* scoreSummary.P95_max_abs_error_cents ...
                   + 0.3 .* scoreSummary.P95_mean_abs_error_cents_last_1s ...
                   + 10  .* scoreSummary.fault_rate;

printDiagnostics(logFile, raw, segmentSummary, scoreSummary);
plotDiagnostics(raw, scoreSummary);

if strlength(saveMatFile) > 0
    save(saveMatFile, 'raw', 'segmentSummary', 'scoreSummary');
end
end

function printDiagnostics(logFile, raw, segmentSummary, scoreSummary)
fprintf('\n=== PITEST IMPORT ===\n');
fprintf('File: %s\n', char(logFile));
fprintf('Rows: %u\n', height(raw));
fprintf('Tests: %u\n', height(scoreSummary));
fprintf('Strings: ');
fprintf('%u ', unique(raw.string_id));
fprintf('\n');

scoreSummary = sortrows(scoreSummary, 'Score');

fprintf('\n=== SCORE SUMMARY ===\n');
for i = 1:height(scoreSummary)
    row = scoreSummary(i, :);
    [status, reason] = classifyCandidate(row);
    fprintf(['%s | str %u | score %.2f | %s\n' ...
             '  lock_fail=%.3f  unlock=%.3f  median_lock=%.0f ms  ' ...
             'P95_max=%.1f cents  P95_last1s=%.1f cents  fault=%.3f\n' ...
             '  reason: %s\n'], ...
            char(row.test_name), ...
            uint32(row.string_id), ...
            row.Score, ...
            status, ...
            row.lock_fail_rate, ...
            row.unlock_rate, ...
            row.median_lock_time_ms, ...
            row.P95_max_abs_error_cents, ...
            row.P95_mean_abs_error_cents_last_1s, ...
            row.fault_rate, ...
            reason);
end

issueMask = (~segmentSummary.lock_success) ...
         | (segmentSummary.unlock_count_after_first_lock > 0) ...
         | (segmentSummary.fault_count > 0);

fprintf('\n=== SEGMENT ISSUES ===\n');
if any(issueMask)
    issues = sortrows(segmentSummary(issueMask, :), {'test_name', 'rep', 'segment_idx'});
    for i = 1:height(issues)
        row = issues(i, :);
        fprintf(['%s | rep %u seg %u | lock=%u unlocks=%u faults=%u ' ...
                 'lock_time=%.0f ms max_err=%.1f cents last1s=%.1f cents trim_rms=%.1f\n'], ...
                char(row.test_name), ...
                uint32(row.rep), ...
                uint32(row.segment_idx), ...
                uint32(row.lock_success), ...
                uint32(row.unlock_count_after_first_lock), ...
                uint32(row.fault_count), ...
                row.lock_time_ms, ...
                row.max_abs_error_cents, ...
                row.mean_abs_error_cents_last_1s, ...
                row.trim_rms);
    end
else
    fprintf('No lock failures, unlocks, or faults were detected.\n');
end
fprintf('\n');
end

function plotDiagnostics(raw, scoreSummary)
if isempty(raw)
    return;
end

if height(scoreSummary) > 1
    plotScoreSummary(scoreSummary);
end

testNames = unique(raw.test_name, 'stable');
for i = 1:numel(testNames)
    testName = testNames(i);
    testRows = raw(raw.test_name == testName, :);
    plotSingleTest(testRows, scoreSummary);
end
drawnow;
end

function plotScoreSummary(scoreSummary)
scoreSummary = sortrows(scoreSummary, 'Score');
labels = cellstr(scoreSummary.test_name);

figure('Name', 'PI Test Score Summary', 'NumberTitle', 'off');
bar(scoreSummary.Score, 0.6);
grid on;
ylabel('Score (lower is better)');
title('PI Test Candidate Ranking');
ax = gca;
ax.XTick = 1:numel(labels);
ax.XTickLabel = labels;
if isprop(ax, 'XTickLabelRotation')
    ax.XTickLabelRotation = 30;
end
end

function plotSingleTest(testRows, scoreSummary)
if isempty(testRows)
    return;
end

testName = testRows.test_name(1);
scoreRow = scoreSummary(scoreSummary.test_name == testName, :);
if isempty(scoreRow)
    scoreText = '';
    status = 'UNKNOWN';
else
    [status, ~] = classifyCandidate(scoreRow(1, :));
    scoreText = sprintf(' | score %.2f', scoreRow.Score(1));
end

t = (testRows.timestamp_ms - testRows.timestamp_ms(1)) / 1000.0;
measuredPlot = testRows.measured_hz;
measuredPlot(measuredPlot <= 0) = NaN;
faultPlot = double(testRows.faults ~= 0);

boundaryMask = false(height(testRows), 1);
boundaryMask(2:end) = diff(testRows.rep) ~= 0 | diff(testRows.segment_idx) ~= 0;
boundaryTimes = t(boundaryMask);

figure('Name', char(testName), 'NumberTitle', 'off');

subplot(4, 1, 1);
plot(t, testRows.target_hz, 'k--', 'LineWidth', 1.2); hold on;
plot(t, measuredPlot, 'b', 'LineWidth', 1.2);
grid on;
ylabel('Hz');
title(sprintf('%s%s | %s', char(testName), scoreText, status), 'Interpreter', 'none');
legend({'Target', 'Measured'}, 'Location', 'best');
drawBoundaries(boundaryTimes);

subplot(4, 1, 2);
plot(t, testRows.error_cents, 'r', 'LineWidth', 1.1); hold on;
plot(t,  10 * ones(size(t)), 'g--');
plot(t, -10 * ones(size(t)), 'g--');
grid on;
ylabel('Error (cents)');
legend({'Error', '+10c', '-10c'}, 'Location', 'best');
drawBoundaries(boundaryTimes);

subplot(4, 1, 3);
plot(t, testRows.trim_steps, 'm', 'LineWidth', 1.1); hold on;
plot(t, testRows.cmd_steps, 'b', 'LineWidth', 1.0);
plot(t, testRows.cur_steps, 'k', 'LineWidth', 1.0);
grid on;
ylabel('Steps');
legend({'Trim', 'Cmd', 'Current'}, 'Location', 'best');
drawBoundaries(boundaryTimes);

subplot(4, 1, 4);
stairs(t, double(testRows.lock ~= 0), 'g', 'LineWidth', 1.1); hold on;
stairs(t, double(testRows.pi_en ~= 0), 'b--', 'LineWidth', 1.0);
stairs(t, faultPlot, 'r', 'LineWidth', 1.0);
grid on;
ylim([-0.1 1.2]);
xlabel('Time (s)');
ylabel('State');
legend({'Lock', 'PI Enable', 'Fault'}, 'Location', 'best');
drawBoundaries(boundaryTimes);
end

function drawBoundaries(boundaryTimes)
if isempty(boundaryTimes)
    return;
end

yl = ylim;
for i = 1:numel(boundaryTimes)
    plot([boundaryTimes(i) boundaryTimes(i)], yl, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
end
ylim(yl);
end

function [status, reason] = classifyCandidate(row)
if row.fault_rate > 0
    status = 'REJECT';
    reason = 'Fault flags were raised during one or more segments.';
elseif row.lock_fail_rate > 0
    status = 'REJECT';
    reason = 'At least one segment failed to achieve stable lock.';
elseif row.unlock_rate > 0
    status = 'REJECT';
    reason = 'The controller locked and then lost lock during one or more segments.';
elseif row.P95_mean_abs_error_cents_last_1s > 25
    status = 'REVIEW';
    reason = 'Stable lock was achieved, but late-segment error is still above the 25-cent feed-forward threshold.';
elseif row.P95_max_abs_error_cents > 50
    status = 'REVIEW';
    reason = 'Transient pitch error is high even though the controller remained stable.';
else
    status = 'PASS';
    reason = 'No failures, unlocks, or faults were detected, and the error metrics are within the target bands.';
end
end

function tf = segmentLockSuccess(lock)
lock = lock(:) ~= 0;
if numel(lock) < 3
    tf = false;
    return;
end
tf = any(conv(double(lock), ones(3, 1), 'valid') == 3);
end

function t = segmentLockTime(timestamp_ms, lock)
idx = firstStableLockIndex(lock);
if isnan(idx)
    t = NaN;
else
    t = timestamp_ms(idx) - timestamp_ms(1);
end
end

function v = segmentMaxAbsError(absErrorCents)
v = maxNaN(absErrorCents);
end

function v = segmentMeanLast1s(timestamp_ms, absErrorCents)
windowMask = lastSecondMask(timestamp_ms);
v = meanNaN(absErrorCents(windowMask));
end

function v = segmentStdLast1s(timestamp_ms, errorCents)
windowMask = lastSecondMask(timestamp_ms);
vals = errorCents(windowMask);
vals = vals(~isnan(vals));
if isempty(vals)
    v = NaN;
else
    v = std(vals, 0);
end
end

function count = segmentUnlockCount(lock)
idx = firstStableLockIndex(lock);
if isnan(idx)
    count = 0;
    return;
end
lock = lock(:) ~= 0;
postLock = lock(idx:end);
count = sum(postLock(1:end-1) & ~postLock(2:end));
end

function count = segmentFaultCount(faults)
count = nnz(faults ~= 0);
end

function v = segmentTrimRms(trimSteps)
trimSteps = trimSteps(:);
if isempty(trimSteps)
    v = NaN;
else
    v = sqrt(mean(trimSteps .^ 2));
end
end

function idx = firstStableLockIndex(lock)
lock = lock(:) ~= 0;
if numel(lock) < 3
    idx = NaN;
    return;
end

stable = conv(double(lock), ones(3, 1), 'valid') == 3;
first = find(stable, 1, 'first');
if isempty(first)
    idx = NaN;
else
    idx = first;
end
end

function mask = lastSecondMask(timestamp_ms)
timestamp_ms = timestamp_ms(:);
if isempty(timestamp_ms)
    mask = false(0, 1);
    return;
end

cutoff = timestamp_ms(end) - 1000;
mask = timestamp_ms >= cutoff;
end

function v = medianNaN(x)
x = x(~isnan(x));
if isempty(x)
    v = NaN;
else
    v = median(x);
end
end

function v = p95NaN(x)
x = x(~isnan(x));
if isempty(x)
    v = NaN;
else
    v = percentileLocal(x, 95);
end
end

function v = meanNaN(x)
x = x(~isnan(x));
if isempty(x)
    v = NaN;
else
    v = mean(x);
end
end

function v = maxNaN(x)
x = x(~isnan(x));
if isempty(x)
    v = NaN;
else
    v = max(x);
end
end

function v = percentileLocal(x, p)
x = sort(x(:));
n = numel(x);
if n == 0
    v = NaN;
    return;
end
if n == 1
    v = x(1);
    return;
end

idx = 1 + (n - 1) * (p / 100);
lo = floor(idx);
hi = ceil(idx);

if lo == hi
    v = x(lo);
else
    alpha = idx - lo;
    v = x(lo) + alpha * (x(hi) - x(lo));
end
end
