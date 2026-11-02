function result = test_view_truth_index_regression()
%TEST_VIEW_TRUTH_INDEX_REGRESSION Reverse lookup, split truth, and truth-only plots.
old_visibility = get(groot, 'DefaultFigureVisible');
set(groot, 'DefaultFigureVisible', 'off');
previous_figures = findall(groot, 'Type', 'figure');
cleanup = onCleanup(@() restore_figures(old_visibility, previous_figures)); %#ok<NASGU>
cfg = struct('truth_id_split_enabled', false, ...
    'passive_truth_id_split_enabled', false, ...
    'truth_cross_sensor_id_consistent', true, 'metrics_max_print', 0);
events = repmat(event_template(), 3, 1);
est = struct('framework', 'joint_2d3d', 'output', {cell(3, 1)}, 'assoc', {cell(3, 1)});
for k = 1:3
    events(k).t_sec = k;
    events(k).passive = struct('n_meas', 4, 'ids', [7, 8, 9, 0], ...
        'ang', [20+k, 50+k, 80+k, 110; 3, 4, 5, 6], ...
        'kind', ones(1, 4), 't_sec', k*ones(1, 4));
    if k == 1, fragment = 1001; else, fragment = 1002; end
    est.output{k} = [output_point(fragment, k, 2), output_point(1003, k, 2)];
    est.assoc{k} = struct('id', [fragment, 1003], ...
        'type', {{'passive', 'passive'}}, 'meas_index', [1, 2], ...
        'filter_dim', [2, 2], 'input_dim', [2, 2], 'tid', [7, 8]);
end
% Track 1003 touches truth 7 once but its evaluation match remains truth 8.
est.assoc{3}.id = 1003; est.assoc{3}.type = {'passive'};
est.assoc{3}.meas_index = 1; est.assoc{3}.filter_dim = 2;
est.assoc{3}.input_dim = 2; est.assoc{3}.tid = 7;
metrics = evaluate_joint_tracking_metrics(est, events, cfg);
opts = struct('index_mode', 'truth', 'truth_ids', 7, 'cfg', cfg, ...
    'metrics', metrics, 'show_angle', false, 'show_enu', false);
info = view_joint_tracks(est, events, opts);
assert(isequal(info.selected_ids, [1001, 1003, 1002]), ...
    'Truth lookup must include every fragment and the track that touched it.');
assert(all([info.truth_tracks.tid] == 7), 'Unrequested truth leaked into the plot.');
wrong_primary = info.reference([info.reference.track_id] == 1003);
assert(isnan(wrong_primary.truth_key) && all(isnan(wrong_primary.angle(:))), ...
    'A different evaluation match must not become the requested error reference.');

% Full and compact review representations must resolve the same tracks.
review = rmfield(est, 'output');
review.output_history = joint_review_output('pack', est.output);
packed = view_joint_tracks(review, events, opts);
assert(isequal(sort(packed.selected_ids), sort(info.selected_ids)));

opts.truth_ids = [7, 8];
multi = view_joint_tracks(est, events, opts);
assert(isequal(multi.truth_selection.selected_keys, [7, 8]));
opts.truth_ids = [];
all_truth = view_joint_tracks(est, events, opts);
assert(isequal(all_truth.truth_selection.selected_keys, [7, 8, 9]));
for invalid = [0, 999]
    opts.truth_ids = invalid;
    assert_error(@() view_joint_tracks(est, events, opts), 'view_joint_tracks:NoTruth');
end

% Truth without a filter association must still produce AE and time plots,
% including measurements that have no 2-D routing/association record.
opts.truth_ids = 9; opts.show_angle = true;
before = numel(findall(groot, 'Type', 'figure'));
untracked = view_joint_tracks(est, events, opts);
assert(isempty(untracked.selected_ids) && numel(untracked.truth_tracks.angle_t) == 3);
assert(numel(findall(groot, 'Type', 'figure')) == before + 2);
empty_est = struct('output', {cell(3, 1)}, 'assoc', {cell(3, 1)});
empty_opts = rmfield(opts, 'metrics'); empty_opts.show_angle = false;
no_outputs = view_joint_tracks(empty_est, events, empty_opts);
assert(isempty(no_outputs.selected_ids) && ~isempty(no_outputs.truth_tracks));

% Original raw ID 21 is split into two spatial instances; a raw query must
% find both. Instance queries must select only their own output track.
spatial = repmat(event_template(), 2, 1);
space_est = struct('output', {cell(2, 1)}, 'assoc', {cell(2, 1)});
for k = 1:2
    spatial(k).t_sec = k;
    spatial(k).active = struct('n_meas', 1, 'ids', 21, 'has_range', true, ...
        'rae', [1000*k; 30; 4], 'xyz', [20000*k; 100; 100], 't_sec', k);
    space_est.output{k} = output_point(k, k, 3);
    space_est.assoc{k} = struct('id', k, 'type', {{'active'}}, 'meas_index', 1, ...
        'filter_dim', 3, 'input_dim', 3, 'tid', 21);
end
split_cfg = cfg; split_cfg.truth_id_split_enabled = true;
split_cfg.truth_id_split_dist_m = 8000; split_cfg.truth_id_split_max_gap_s = 10;
raw_opts = struct('index_mode', 'truth', 'truth_ids', 21, 'truth_id_type', 'raw', ...
    'cfg', split_cfg, 'show_angle', false, 'show_enu', false);
raw = view_joint_tracks(space_est, spatial, raw_opts);
assert(isequal(raw.selected_ids, [1, 2]) && numel(raw.truth_selection.selected_keys) == 2);
raw_opts.truth_id_type = 'instance'; raw_opts.truth_ids = raw.truth_selection.selected_keys(2);
instance = view_joint_tracks(space_est, spatial, raw_opts);
assert(isequal(instance.selected_ids, 2));
% Exercise position-only plotting with no formal output history.
space_empty = struct('output', {cell(2, 1)}, 'assoc', {cell(2, 1)});
raw_opts.show_enu = true;
before = numel(findall(groot, 'Type', 'figure'));
space_truth = view_joint_tracks(space_empty, spatial, raw_opts);
assert(isempty(space_truth.selected_ids) && numel(findall(groot, 'Type', 'figure')) == before+2);

% The original filter-ID entry point still selects only the requested ID.
track_opts = struct('track_ids', 1001, 'cfg', cfg, 'metrics', metrics, ...
    'show_angle', false, 'show_enu', false);
by_track = view_joint_tracks(est, events, track_opts);
assert(isequal(by_track.selected_ids, 1001));
result = struct('passed', true, 'truth_7_filter_ids', info.selected_ids, ...
    'raw_21_instance_ids', raw.truth_selection.selected_keys, ...
    'truth_only_2d_and_3d', true, 'compact_review_supported', true);
fprintf('真值索引回归通过：多段/触达航迹、实例/原始编号、无滤波真值、无效编号和原滤波索引。\n');
end

function e = event_template()
e = struct('t_sec', 0, 'active', struct('n_meas', 0, 'ids', [], ...
    'has_range', false(1, 0), 'rae', zeros(3, 0), 'xyz', zeros(3, 0), 't_sec', []), ...
    'passive', struct('n_meas', 0, 'ids', [], 'ang', zeros(2, 0), 'kind', [], 't_sec', []));
end

function out = output_point(id, t, dim)
out = struct('id', id, 't_sec', t, 'output_dim', dim, 'az_deg', 20+t, ...
    'el_deg', 3, 'position_enu', nan(3, 1), 'truth_id', NaN);
if dim == 3, out.position_enu = [20000*t; 100; 100]; end
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

function restore_figures(visibility, previous)
created = setdiff(findall(groot, 'Type', 'figure'), previous);
delete(created);
set(groot, 'DefaultFigureVisible', visibility);
end
