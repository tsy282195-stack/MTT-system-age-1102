function result = test_clutter_definition_regression()
%TEST_CLUTTER_DEFINITION_REGRESSION Shared clutter definition and cache refresh.
cfg = struct('truth_id_split_enabled', false, ...
    'passive_truth_id_split_enabled', false, ...
    'truth_cross_sensor_id_consistent', true, 'metrics_max_print', 0);
K = 3;
events = repmat(event_template(), K, 1);
est = struct('framework', 'joint_2d3d', 'output', {cell(K, 1)}, ...
    'assoc', {cell(K, 1)}, 'branch_confirmation_events', ...
    struct('basis', 'test', 'available_dims', [2, 3], ...
    'branch_dim', [2, 2, 2, 3], 'branch_id', [1, 2, 3, 1], ...
    'event_index', [1, 1, 1, 1]));
for k = 1:K
    events(k).t_sec = k;
    events(k).active = struct('n_meas', 2, 'ids', [0, 7], ...
        'has_range', [true, true], 'rae', [1000, 2000; 10, 20; 2, 3], ...
        'xyz', [1000, 2000; 100, 200; 10, 20], 't_sec', [k, k]);
    events(k).passive = struct('n_meas', 4, 'ids', [0, 7, -1, NaN], ...
        'ang', [10, 20, 30, 40; 2, 3, 4, 5], ...
        't_sec', k*ones(1, 4), 'kind', ones(1, 4));
    est.output{k} = struct('id', 1001, 't_sec', k, 'output_dim', 2, ...
        'az_deg', 10, 'el_deg', 2, 'position_enu', nan(3, 1), 'truth_id', NaN);
    est.assoc{k} = struct('id', [1001, 1002, 1003, 1], ...
        'type', {{'passive', 'passive', 'passive', 'active'}}, ...
        'meas_index', [1, 2, 3, 1], 'filter_dim', [2, 2, 2, 3], ...
        'input_dim', [2, 2, 2, 3], 'usage_branch_dim', [2, 2, 2, 3], ...
        'update_applied', true(1, 4), ...
        'branch_2d_id', [1, 2, 3, NaN], 'branch_3d_id', [NaN, NaN, NaN, 1]);
end
original = events;
baseline = evaluate_clutter_acceptance(est, events);
assert(isequaln(baseline, evaluate_clutter_acceptance(est, events, cfg)));
assert(baseline.overall.n_input == 6 && baseline.overall.n_accepted == 6);
assert(baseline.two_d.n_input == 3 && baseline.three_d.n_input == 3);
metrics = evaluate_joint_tracking_metrics(est, events, cfg);
assert(~any(metrics.truth_targets.key_raw_id == 0));

% Replacing zero with a list must also stop treating zero as clutter.
custom = cfg; custom.clutter_raw_ids = [7; -1; 7];
c = evaluate_clutter_acceptance(est, events, custom);
assert(isequal(c.clutter_raw_ids, [-1, 7]));
assert(c.two_d.n_input == 6 && c.three_d.n_input == 3);
assert(c.overall.n_input == 9 && c.overall.n_accepted == 6);
assert(c.overall.n_init_success == 2 && c.overall.n_update == 4);
assert(c.overall.n_not_used == 3);
custom_metrics = evaluate_joint_tracking_metrics(est, events, custom);
assert(isequal(custom_metrics.truth_targets.key_raw_id, 0));
assert(custom_metrics.track_details([custom_metrics.track_details.track_id] == 1001).two_d.matched_truth_id == 0);

% An explicit empty list must survive every path, including truth splitting.
disabled = cfg; disabled.clutter_raw_ids = [];
for split = [false, true]
    for consistent = [false, true]
        disabled.truth_id_split_enabled = split;
        disabled.truth_cross_sensor_id_consistent = consistent;
        m = evaluate_joint_tracking_metrics(est, events, disabled);
        assert(any(m.truth_targets.key_raw_id == 0));
        assert(~m.clutter_acceptance.definition_enabled && m.clutter_acceptance.n_input == 0);
        assert(strcmp(m.clutter_acceptance.status, 'clutter_definition_disabled'));
        assert(isnan(m.clutter_acceptance.acceptance_rate));
        assert(m.overall.association.n_measurements == metrics.overall.association.n_measurements);
    end
end
assert(isequaln(events, original), 'Evaluation must not mutate filter inputs.');
disabled = cfg; disabled.clutter_raw_ids = [];
disabled_metrics = evaluate_joint_tracking_metrics(est, events, disabled);
detail = disabled_metrics.track_details([disabled_metrics.track_details.track_id] == 1001);
assert(detail.two_d.matched_truth_id == 0 && detail.two_d.n_correct == K);
message = evalc('print_clutter_acceptance(disabled_metrics.clutter_acceptance);');
assert(contains(message, '已关闭按编号定义杂波'));

% A changed definition must invalidate supplied metrics in viewers/reports.
opts = struct('index_mode', 'truth', 'truth_id_type', 'raw', 'truth_ids', 0, ...
    'cfg', disabled, 'metrics', metrics, 'show_angle', false, 'show_enu', false);
info = view_joint_tracks(est, events, opts);
assert(isequal(info.selected_ids, 1001) && info.reference.truth_key == 0);
opts.truth_id_type = 'instance';
info_instance = view_joint_tracks(est, events, opts);
assert(isequal(info_instance.selected_ids, 1001));
opts.cfg = cfg; opts.metrics = disabled_metrics;
assert_error(@() view_joint_tracks(est, events, opts), 'view_joint_tracks:NoTruth');
opts.cfg = custom; opts.truth_id_type = 'raw'; opts.truth_ids = 7;
assert_error(@() view_joint_tracks(est, events, opts), 'view_joint_tracks:NoTruth');

report_dir = tempname;
report_opts = struct('output_dir', report_dir, 'print_console', false, 'metrics', metrics);
report = export_track_diagnostics(est, events, disabled, 1001, report_opts);
assert(report.n_reports == 1);
assert(contains(fileread(report.files{1}), '已关闭按编号定义杂波'));

% Validate IDs early, rather than silently accepting malformed definitions.
assert(isequal(clutter_raw_ids(struct()), 0));
assert(isempty(clutter_raw_ids(disabled)));
for invalid = {NaN, Inf, 0.5, [0, 1; 2, 3], '0', true, 1i}
    bad = struct('clutter_raw_ids', invalid{1});
    assert_error(@() clutter_raw_ids(bad), 'clutter_raw_ids:InvalidConfig');
    assert_error(@() validate_config_fusion(bad), 'clutter_raw_ids:InvalidConfig');
end
result = struct('passed', true, 'default_input', baseline.overall.n_input, ...
    'custom_input', c.overall.n_input, 'disabled_zero_is_truth', true, ...
    'viewer_and_report_cache_refreshed', true, 'report_dir', report_dir);
fprintf('杂波定义回归通过：默认0、自定义多个编号、空列表关闭、真值分割、查看/报告缓存刷新及配置校验。\n');
end

function e = event_template()
e = struct('t_sec', 0, 'active', struct(), 'passive', struct());
end

function assert_error(f, identifier)
caught = false;
try
    f();
catch ME
    caught = strcmp(ME.identifier, identifier);
    if ~caught, rethrow(ME); end
end
assert(caught, 'Expected error %s.', identifier);
end
