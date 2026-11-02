function result = test_posterior_noise_regression(legacy_dir)
%TEST_POSTERIOR_NOISE_REGRESSION Sensor isolation and causal posterior-R updates.
if nargin < 1, legacy_dir = ''; end
addpath(fileparts(mfilename('fullpath')));
cfg = test_config();
pa = posterior_noise_params(cfg, 'active', 'angle');
pp = posterior_noise_params(cfg, 'passive', 'angle');
assert(abs(pa.threshold - 9.21034037197618) < 1e-10);
assert(pp.threshold > pa.threshold && pp.d_min ~= pa.d_min);

% The normal and outlier cases have analytic expectations, including full
% off-diagonal covariance and the exact d_min saturation boundary.
R0 = [2, 0.3; 0.3, 1]; e = [0.2; -0.3]; C = [0.5, 0.1; 0.1, 0.4];
[bank, R1, normal] = posterior_noise_update(struct(), R0, e, C, [1; 1], eye(2), pa);
assert(normal.updated && normal.d == 1 && norm(R1 - (e * e' + C), 'fro') < 1e-12);
nu = [10; 0]; expected_d = pa.threshold / 100;
[~, R2, outlier] = posterior_noise_update(struct(), R0, e, C, nu, eye(2), pa);
assert(abs(outlier.d - expected_d) < 1e-12);
assert(norm(R2 - ((1 - expected_d)*R0 + expected_d*(e*e' + C)), 'fro') < 1e-12);
[~, ~, extreme] = posterior_noise_update(struct(), R0, e, C, [1e6; 0], eye(2), pa);
assert(extreme.d == pa.d_min);
disabled = pa; disabled.enabled = false;
[bank_off, R_off, info_off] = posterior_noise_update(bank, R0, e, C, nu, eye(2), disabled);
assert(isequaln(bank_off, bank) && isequal(R_off, R0) && ~info_off.updated);
assert(isequal(posterior_noise_get(bank, R0, disabled), R0));
[~, R_floor] = posterior_noise_update(struct(), R0, zeros(2, 1), zeros(2), ...
    zeros(2, 1), eye(2), pp);
assert(all(diag(R_floor) >= pp.min_diag) && all(eig(R_floor) > 0));
[bad_bank, ~, bad] = posterior_noise_update(bank, R0, [NaN; 0], C, nu, eye(2), pa);
assert(~bad.updated && isequaln(bad_bank, bank));

events = angle_events();
switches = logical([0, 0; 1, 0; 0, 1; 1, 1]);
for k = 1:4
    c = cfg; c.active_adapt_R_enabled = switches(k, 1);
    c.passive_adapt_R_enabled = switches(k, 2);
    [~, state] = run_filter_joint_2d3d(events, [], c);
    assert(numel(state.tracks) == 1, 'Angle fixture must retain one logical track.');
    b = state.tracks(1).angle.noise;
    assert(isfield(b, 'active_ae') == switches(k, 1));
    assert(isfield(b, 'passive_ae') == switches(k, 2));
    if switches(k, 1)
        assert(b.active_ae.n_updates >= 2 && b.active_ae.info.threshold == pa.threshold);
    end
    if switches(k, 2)
        assert(b.passive_ae.n_updates >= 2 && b.passive_ae.info.threshold == pp.threshold);
    end
end
% Cache/streaming paths must agree, including across +/-180 degrees.
[~, batch] = run_filter_joint_2d3d(events, [], cfg);
c = cfg; c.joint_projection_cache_enabled = false;
[~, uncached] = run_filter_joint_2d3d(events, [], c);
assert(norm(batch.tracks(1).angle.x - uncached.tracks(1).angle.x) < 1e-10);
stream = [];
for k = 1:numel(events)
    [~, stream] = run_filter_joint_2d3d(events(k), [], cfg, stream);
end
assert(norm(batch.tracks(1).angle.x - stream.tracks(1).angle.x) < 1e-10);
assert(isequaln(batch.tracks(1).angle.noise, stream.tracks(1).angle.noise));
assert(abs(batch.tracks(1).angle.noise.passive_ae.info.posterior_residual(1)) < 1);
% Disabling a sensor after resumption must stop reading/learning its saved R.
c = cfg; c.passive_adapt_R_enabled = false;
next = events(end); next.t_sec = next.t_sec + 0.1; next.passive.t_sec = next.t_sec;
next.passive.kind = 1;
old_count = stream.tracks(1).angle.noise.passive_ae.n_updates;
[~, resumed] = run_filter_joint_2d3d(next, [], c, stream);
assert(resumed.tracks(1).angle.noise.passive_ae.n_updates == old_count);

% Exercise the direct spatial branch (active XYZ and both angle sources).
spatial = events;
for k = 1:2
    spatial(k).has_active = true; spatial(k).has_passive = false;
    spatial(k).passive = empty_passive();
    xyz = [1000 + k; 10000 + 2*k; 1000];
    spatial(k).active = active_packet(xyz, spatial(k).t_sec);
end
for k = 3:numel(spatial)
    spatial(k).passive.ang = [atan2d(1002, 10004); atan2d(1000, hypot(1002, 10004))] ...
        + [0.002*sin(k); 0.001*cos(k)];
end
[~, spatial_state] = run_filter_joint_2d3d(spatial, [], cfg);
assert(numel(spatial_state.tracks) == 1);
bs = spatial_state.tracks(1).space.noise;
assert(all(isfield(bs, {'active_xyz', 'active_ae', 'passive_ae'})));
% Active AE-only can also arrive directly in the active packet collection.
direct_ae = events;
for k = 2:2:numel(direct_ae)
    angle = direct_ae(k).passive.ang;
    direct_ae(k).has_active = true; direct_ae(k).has_passive = false;
    direct_ae(k).active = active_packet([1000; 10000; 1000], direct_ae(k).t_sec);
    direct_ae(k).active.rae = [NaN; angle]; direct_ae(k).active.has_range = false;
    direct_ae(k).passive = empty_passive();
end
[~, direct_state] = run_filter_joint_2d3d(direct_ae, [], cfg);
assert(isfield(direct_state.tracks(1).angle.noise, 'active_ae'));

% Mature 3-D backbone: multi-echo scaling plus active/passive bearing updates.
[xyz, R, times, bearings, platform, ids, meta] = backbone_inputs();
for k = 1:4
    c = cfg; c.active_adapt_R_enabled = switches(k, 1);
    c.passive_adapt_R_enabled = switches(k, 2);
    text = evalc('est = run_filter_adapt_ckf(xyz, R, times, c, bearings, platform, ids, meta);'); %#ok<NASGU>
    assert(size(est.tracks{end}.m, 2) == 1, 'Backbone fixture must retain one track.');
    b = est.tracks{end}.measurement_noise{1};
    assert(isfield(b, 'active_xyz') == switches(k, 1));
    assert(isfield(b, 'active_ae') == switches(k, 1));
    assert(isfield(b, 'passive_ae') == switches(k, 2));
    assert(all(isfinite(est.tracks{end}.m(:))));
    if switches(k, 1)
        assert(all(eig(b.active_xyz.R) > 0));
        % Exact accepted group: fused posterior covariance must be mapped
        % back to raw-echo units rather than fed through R_scale twice.
        assert(b.active_xyz.n_updates == numel(times) - 1);
    end
end
% The normal review result retains compact final R/M/d snapshots as well.
c = cfg; c.result_save_mode = 'review';
captured = evalc('review = run_filter_joint_legacy_backbone(xyz, R, times, bearings, platform, ids, meta, c, meta);'); %#ok<NASGU>
assert(review.measurement_noise.active_enabled && review.measurement_noise.passive_enabled);
assert(~isempty(review.measurement_noise.backbone));
assert(isfield(review.measurement_noise.backbone(1).bank, 'active_xyz'));
disabled_matches_legacy = [];
if ~isempty(legacy_dir)
    c = cfg; c.active_adapt_R_enabled = false; c.passive_adapt_R_enabled = false;
    c.adapt_R_enabled = false; c.adapt_R_alpha = 0.4;
    c.adapt_R_window = 50; c.adapt_R_min_samples = 25;
    captured = evalc('new3 = run_filter_adapt_ckf(xyz, R, times, c, bearings, platform, ids, meta);'); %#ok<NASGU>
    [new2, ~] = run_filter_joint_2d3d(events, [], c);
    original_dir = pwd; cleanup = onCleanup(@() cd(original_dir));
    cd(legacy_dir); clear run_filter_adapt_ckf run_filter_joint_2d3d
    captured = evalc('old3 = run_filter_adapt_ckf(xyz, R, times, c, bearings, platform, ids, meta);'); %#ok<NASGU>
    [old2, ~] = run_filter_joint_2d3d(events, [], c);
    cd(original_dir); clear run_filter_adapt_ckf run_filter_joint_2d3d
    clear cleanup
    assert(isequaln(old3.X, new3.X) && isequaln(old3.P, new3.P));
    assert(isequaln(old3.tracks{end}.m, new3.tracks{end}.m));
    assert(isequaln(old3.tracks{end}.P, new3.tracks{end}.P));
    assert(isequaln(old2.X2, new2.X2) && isequaln(old2.P2, new2.P2));
    assert(isequaln(old2.assoc, new2.assoc));
    disabled_matches_legacy = true;
end
result = struct('passed', true, 'switch_combinations', 4, ...
    'disabled_matches_legacy', disabled_matches_legacy, ...
    'active_chi2_2d', pa.threshold, 'passive_chi2_2d', pp.threshold, ...
    'angle_updates', batch.tracks(1).angle.noise, 'spatial_updates', bs);
fprintf('Posterior-R regressions passed: sensor switches, wrap, streaming/cache, XYZ/bearing/fusion.\n');
end

function cfg = test_config()
cfg = config_fusion();
cfg.active_adapt_R_min_std_xyz_m = [0.5, 0.5, 0.5];
cfg.R_min_diag = [1, 1, 1];
cfg.joint_confirm_M = 1; cfg.joint_confirm_N = 1;
cfg.joint_passive_confirm_group_size = 1;
cfg.joint_passive_confirmation_cycles_only = false;
cfg.joint_streaming_quiet = true; cfg.joint_los_trend_enabled = false;
cfg.joint_2d_rate_cost_enabled = false; cfg.joint_2d_stable_lock_enabled = false;
cfg.joint_2d_ambiguity_hold_enabled = false;
cfg.assoc_quality_mode = 'off'; cfg.track_quality_mode = 'off';
cfg.bearing_robust_mode = 'off'; cfg.joint_angle95_max_deg = 10;
cfg.joint_gate_2d = 100; cfg.joint_2d_accept_nis = 100;
cfg.joint_2d_max_los_separation_deg = 20;
cfg.joint_extension_enabled = false; cfg.vel_trend_enabled = false;
cfg.use_imm = false; cfg.M_confirm = 2; cfg.N_confirm = 3;
cfg.passive_bearing_update_on_active = true;
cfg.passive_bearing_update_active_hit_tracks = true;
cfg.passive_bearing_min_dt_s = 0; cfg.passive_bearing_fast_gate_deg = 10;
cfg.passive_bearing_gate = 100; cfg.nis_gate = 200; cfg.assoc_accept_nis = 200;
cfg.meas_fuse_weight = 'equal'; cfg.meas_fuse_cluster_dist_m = 100;
cfg.tentative_max_silence_s = 10; cfg.confirmed_max_silence_s = 10;
end

function events = angle_events()
empty_a = active_packet([], 0);
events = repmat(struct('cycle_id', 0, 't_sec', 0, 't_start', 0, 't_end', 0, ...
    'confirm_cycle', true, 'has_active', false, 'has_passive', true, ...
    'active', empty_a, 'passive', empty_passive()), 8, 1);
for k = 1:8
    t = (k - 1)*0.1; events(k).cycle_id = k;
    events(k).t_sec = t; events(k).t_start = t; events(k).t_end = t;
    az = mod(179.95 + 0.02*k + 180, 360) - 180;
    events(k).passive = struct('t_sec', t, 'ang', [az; 1 + 0.002*sin(k)], ...
        'R_ae', diag([0.1, 0.05].^2), 'ids', 1, 'src', 1, ...
        'kind', 1 + double(mod(k, 2) == 0), 'original_index', k, 'n_meas', 1);
end
end

function a = active_packet(xyz, t)
a = struct('t_sec', zeros(1, 0), 'xyz', zeros(3, 0), 'rae', zeros(3, 0), ...
    'R_xyz', zeros(3, 3, 0), 'R_ae', zeros(2, 2, 0), 'has_range', false(1, 0), ...
    'ids', zeros(1, 0), 'src', zeros(1, 0), 'n_meas', 0);
if isempty(xyz), return; end
a.t_sec = t; a.xyz = xyz; a.has_range = true;
a.rae = [norm(xyz); atan2d(xyz(1), xyz(2)); atan2d(xyz(3), hypot(xyz(1), xyz(2)))];
a.R_xyz = diag([100, 140, 80].^2); a.R_ae = diag([0.1, 0.05].^2);
a.ids = 1; a.src = 1; a.n_meas = 1;
end

function p = empty_passive()
p = struct('t_sec', zeros(1, 0), 'ang', zeros(2, 0), 'R_ae', zeros(2, 2, 0), ...
    'ids', zeros(1, 0), 'src', zeros(1, 0), 'kind', zeros(1, 0), ...
    'original_index', zeros(1, 0), 'n_meas', 0);
end

function [xyz, R, times, bearing, platform, ids, meta] = backbone_inputs()
times = (0:7)' * 0.1; n = numel(times);
xyz = cell(n, 1); R = cell(n, 1); ids = cell(n, 1); bearing = cell(n, 1);
meta = repmat(struct('has_active', true, 'has_passive', true, ...
    'miss_cycle', true, 'confirm_cycle', true, 'n_active', 2, 'n_passive', 2, ...
    't_start', 0, 't_end', 0), n, 1);
platform = struct('lat_deg', 30, 'lon_deg', 120, 'alt_m', 100, ...
    'interp_lat', @(t)30 + zeros(size(t)), 'interp_lon', @(t)120 + zeros(size(t)), ...
    'interp_alt', @(t)100 + zeros(size(t)));
for k = 1:n
    z = [1000 + 2*k; 10000 + 3*k; 1000];
    xyz{k} = [z, z + [2; -1; 1]];
    R{k} = repmat(diag([100, 140, 80].^2), 1, 1, 2); ids{k} = [1, 1];
    if k == 1
        xyz{k} = z; R{k} = R{k}(:, :, 1); ids{k} = 1; meta(k).n_active = 1;
    end
    ae = [atan2d(z(1), z(2)); atan2d(z(3), hypot(z(1), z(2)))];
    bearing{k} = struct('ang_deg', [ae + [0.002; -0.001], ae + [-0.003; 0.002]], ...
        'R_deg2', repmat(diag([0.1, 0.05].^2), 1, 1, 2), ...
        'tracklet_id', [1, 1], 'src', [1, 2], 'kind', [1, 2], 'n_meas', 2);
    meta(k).t_start = times(k); meta(k).t_end = times(k);
end
end
