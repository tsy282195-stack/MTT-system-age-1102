function result = test_passive_truth_split_regression()
%TEST_PASSIVE_TRUTH_SPLIT_REGRESSION Time-only split for pure-passive truth.

cfg = struct('truth_id_split_enabled', false, ...
    'passive_truth_id_split_enabled', true, ...
    'passive_truth_id_split_max_gap_s', 10, ...
    'truth_cross_sensor_id_consistent', true);

events = [make_angle_event(0, 7, 1), ...
    make_angle_event(10, 7, 1), make_angle_event(20.1, 7, 1)];
labels = build_joint_truth_labels(events, cfg);
keys = cellfun(@(x) x(1), labels.passive);
assert(keys(1) == keys(2) && keys(3) ~= keys(2), ...
    'Only a consecutive physical-passive AE time gap greater than 10 s may split truth.');
assert(labels.summary.n_passive_time_split_raw_ids == 1, ...
    'The passive split summary must count the split raw truth ID.');

events_equal = [make_angle_event(0, 8, 1), ...
    make_angle_event(10, 8, 1), make_angle_event(20, 8, 1)];
labels_equal = build_joint_truth_labels(events_equal, cfg);
keys_equal = cellfun(@(x) x(1), labels_equal.passive);
assert(numel(unique(keys_equal)) == 1, ...
    'A gap exactly equal to the threshold must not split truth.');

cfg.passive_truth_id_split_enabled = false;
labels_off = build_joint_truth_labels(events, cfg);
keys_off = cellfun(@(x) x(1), labels_off.passive);
assert(numel(unique(keys_off)) == 1, ...
    'The passive truth split switch must fully disable time splitting.');

cfg.passive_truth_id_split_enabled = true;
active_angle_events = [make_angle_event(0, 9, 2), ...
    make_angle_event(20.1, 9, 2)];
labels_active_angle = build_joint_truth_labels(active_angle_events, cfg);
keys_active_angle = cellfun(@(x) x(1), labels_active_angle.passive);
assert(numel(unique(keys_active_angle)) == 1, ...
    'Active-file AE-only truth must not be split by the physical-passive switch.');

% Exercise the printed metrics report as well as label construction. This
% catches accidental calls to local helpers that are not defined inside the
% evaluation file (the former field_or visibility regression).
metrics_cfg = cfg;
metrics_cfg.metrics_max_print = 1;
empty_est = struct();
empty_est.assoc = cell(numel(events), 1);
empty_est.output = cell(numel(events), 1);
empty_est.logical_tracks = cell(numel(events), 1);
empty_est.transition_log = struct( ...
    'id', {}, 't_sec', {}, 'from', {}, 'to', {}, 'reason', {});
metrics = evaluate_joint_tracking_metrics(empty_est, events, metrics_cfg);
assert(strcmp(metrics.status, 'ok') && ...
    metrics.two_d.association.n_measurements == numel(events), ...
    'The metrics report must run with pure-passive truth splitting enabled.');

result = struct('passed', true, 'split_keys', keys, ...
    'equal_threshold_keys', keys_equal, 'switch_off_keys', keys_off, ...
    'metrics_report_status', metrics.status);
fprintf('纯被动真值时间拆分回归通过: >10s拆分, =10s不拆分, 关闭开关不拆分。\n');
end

function e = make_angle_event(t, raw_id, kind)
e = struct();
e.t_sec = t;
e.active = struct('n_meas', 0, 'ids', zeros(1, 0), ...
    'xyz', zeros(3, 0), 'rae', zeros(3, 0), ...
    'has_range', false(1, 0), 't_sec', zeros(1, 0));
e.passive = struct('n_meas', 1, 'ids', raw_id, ...
    'ang', [20; 3], 'kind', kind, 't_sec', t);
end
