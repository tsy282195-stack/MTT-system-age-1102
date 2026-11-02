function result = test_confirmed_lock_scope_regression(baseline_file)
% Exercise the production private assignment function with controlled costs.
% MATLAB private local functions need a temporary entry point; their source
% is read verbatim from the kernel, rather than reimplemented in this test.
if nargin < 1, baseline_file = ''; end
root = fileparts(mfilename('fullpath'));
folder = fullfile(tempdir, ['confirmed_lock_test_', char(java.util.UUID.randomUUID)]);
mkdir(folder);
cleanup = onCleanup(@() cleanup_probe(folder)); %#ok<NASGU>
write_probe(fullfile(root, 'run_filter_joint_2d3d.m'), folder, 'lock_scope_current');
if ~isempty(baseline_file)
    write_probe(baseline_file, folder, 'lock_scope_before');
end
addpath(folder, '-begin');

tr = repmat(struct('confirmed', true, 'mode', '2d', ...
    'space', struct('valid', false)), 1, 3);
tr(3).confirmed = false;
p = struct('range_assist', struct('enabled', false, 'weight', 0), ...
    'stable_lock_enabled', true, 'lock_margin', 1, ...
    'lock_max_az_residual_deg', .5, 'lock_max_rate_cost', 4, ...
    'ambiguity_hold_enabled', false, 'ambiguity_max_events', 0, ...
    'unmatched_cost', 250);
A = [1.5, 21.4; 16.4, 39.9; 1.99, 2.67];
r = ranking(A);
[pairs, info, final] = lock_scope_current(A, tr, r, p, [], 0, zeros(3, 1));
assert_pair(pairs, 1, 1); assert_pair(pairs, 2, 2);
assert(info.n_stable_locked == 1 && isequaln(A, final));
checks = {'tentative_near_cost_does_not_veto_lock'};
if ~isempty(baseline_file)
    old = lock_scope_before(A, tr, r, p, [], 0, zeros(3, 1));
    assert_pair(old, 1, 2); assert_pair(old, 2, 1);
    checks{end + 1} = 'original_identity_exchange_reproduced';
end

B = A; B(3, 1) = .1;
[pairs, info] = lock_scope_current(B, tr, r, p, [], 0, zeros(3, 1));
assert_pair(pairs, 1, 1); assert(info.n_stable_locked == 1);
checks{end + 1} = 'cheaper_tentative_does_not_veto_column_best';

B = A; B(2, :) = [1.9, 3];
[~, info] = lock_scope_current(B, tr, r, p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0);
checks{end + 1} = 'confirmed_competitor_still_vetoes_lock';
space = tr; space(2).mode = '3d'; space(2).space.valid = true;
[~, info] = lock_scope_current(B, space, r, p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0);
checks{end + 1} = 'confirmed_3d_competitor_is_preserved';

B = [A, [inf; inf; .1]];
[pairs, info, final] = lock_scope_current(B, tr, ranking(B), p, [], 0, zeros(3, 1));
assert_pair(pairs, 1, 1); assert_pair(pairs, 2, 2); assert_pair(pairs, 3, 3);
assert(info.n_stable_locked == 1 && isequaln(B, final));
assert(numel(unique(pairs(:, 2))) == size(pairs, 1));
checks{end + 1} = 'tentative_keeps_leftovers_and_measurements_are_unique';

r.az_residual_deg(1, 1) = .51;
[~, info] = lock_scope_current(A, tr, r, p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0);
r = ranking(A); r.rate_cost(1, 1) = 4.01;
[~, info] = lock_scope_current(A, tr, r, p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0);
checks{end + 1} = 'azimuth_and_rate_limits_remain_required';

B = [1.5, 1.6; 10, .1]; two = tr([1, 3]);
[~, info] = lock_scope_current(B, two, ranking(B), p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0);
checks{end + 1} = 'own_measurement_alternatives_still_limit_margin';

tentative = tr; [tentative.confirmed] = deal(false);
[pairs, info] = lock_scope_current(A, tentative, ranking(A), p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0 && ~isempty(pairs));
checks{end + 1} = 'all_tentative_still_assign_without_locks';

B = inf(3, 2);
[pairs, info] = lock_scope_current(B, tr, ranking(B), p, [], 0, zeros(3, 1));
assert(isempty(pairs) && info.n_stable_locked == 0);
B = [1.5, inf; 16.4, inf; .1, inf];
[pairs, info] = lock_scope_current(B, tr, ranking(B), p, [], 0, zeros(3, 1));
assert_pair(pairs, 1, 1); assert(info.n_stable_locked == 1);
checks{end + 1} = 'empty_candidates_and_unavailable_columns';

formal = tr; [formal.confirmed] = deal(true);
[pairs, info, final] = lock_scope_current(A, formal, ranking(A), p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0);
if ~isempty(baseline_file)
    [old, old_info, old_final] = lock_scope_before(A, formal, ranking(A), p, [], 0, zeros(3, 1));
    assert(isequaln(pairs, old) && isequaln(info, old_info) && isequaln(final, old_final));
end
checks{end + 1} = 'all_confirmed_behavior_unchanged';
p.stable_lock_enabled = false;
[pairs, info, final] = lock_scope_current(A, tr, ranking(A), p, [], 0, zeros(3, 1));
assert(info.n_stable_locked == 0); assert_pair(pairs, 1, 2);
if ~isempty(baseline_file)
    [old, old_info, old_final] = lock_scope_before(A, tr, ranking(A), p, [], 0, zeros(3, 1));
    assert(isequaln(pairs, old) && isequaln(info, old_info) && isequaln(final, old_final));
end
checks{end + 1} = 'disabled_lock_behavior_unchanged';
result = struct('passed', true, 'checks', {checks}, 'n_checks', numel(checks));
fprintf('CONFIRMED_LOCK_REGRESSION %s\n', jsonencode(result));
end

function r = ranking(A)
r = struct('az_residual_deg', zeros(size(A)), 'rate_cost', zeros(size(A)));
end

function assert_pair(pairs, i, j)
assert(nnz(pairs(:, 1) == i & pairs(:, 2) == j) == 1, ...
    'Expected track row %d to receive measurement column %d.', i, j);
end

function write_probe(source_file, folder, name)
source = fileread(source_file);
starts = regexp(source, '(?m)^function[ \t]');
assert(numel(starts) > 1);
fid = fopen(fullfile(folder, [name, '.m']), 'w', 'n', 'UTF-8');
assert(fid >= 0);
closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'function varargout=%s(varargin)\n', name);
fprintf(fid, '[varargout{1:nargout}]=protected_passive_assignment(varargin{:});\nend\n');
fprintf(fid, '%s', source(starts(2):end));
end

function cleanup_probe(folder)
clear lock_scope_current lock_scope_before;
if contains([path, pathsep], [folder, pathsep]), rmpath(folder); end
for name = {'lock_scope_current.m', 'lock_scope_before.m'}
    file = fullfile(folder, name{1});
    if isfile(file), delete(file); end
end
if isfolder(folder), rmdir(folder); end
end
