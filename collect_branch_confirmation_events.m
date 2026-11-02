function history = collect_branch_confirmation_events(space_est, angle_est, unified_angle)
%COLLECT_BRANCH_CONFIRMATION_EVENTS First actual confirmation, never output time.
% Called before internal histories are compacted or public IDs are remapped.
if nargin < 1, space_est = struct(); end
if nargin < 2, angle_est = struct(); end
if nargin < 3, unified_angle = false; end
history = struct('basis', 'first_confirmed_internal_snapshot', ...
    'available_dims', zeros(1, 0), 'branch_dim', zeros(1, 0), ...
    'branch_id', zeros(1, 0), 'event_index', zeros(1, 0));
if isfield(space_est, 'tracks') && iscell(space_est.tracks)
    history.available_dims(end+1) = 3;
    for k = 1:numel(space_est.tracks)
        s = space_est.tracks{k};
        if ~isstruct(s) || ~isfield(s, 'conf') || ~isfield(s, 'L') || size(s.L,1)<2, continue; end
        n = min(numel(s.conf), size(s.L,2));
        ids = s.L(2, find(s.conf(1:n)==1)); %#ok<FNDSB>
        history = remember(history, ids, 3, k);
    end
end
if isfield(angle_est, 'logical_tracks') && iscell(angle_est.logical_tracks)
    dims = 2; if unified_angle, dims = [2,3]; end
    history.available_dims = unique([history.available_dims,dims]);
    for k = 1:numel(angle_est.logical_tracks)
        s = angle_est.logical_tracks{k};
        if isempty(s) || ~isstruct(s) || ~isfield(s,'confirmed') || ~isfield(s,'id'), continue; end
        ids = [s(logical([s.confirmed])).id];
        for dim = dims, history = remember(history, ids, dim, k); end
    end
end
end

function history = remember(history, ids, dim, k)
ids = unique(ids(isfinite(ids) & ids>0));
known = history.branch_id(history.branch_dim==dim);
ids = ids(~ismember(ids,known));
history.branch_dim = [history.branch_dim,repmat(dim,1,numel(ids))];
history.branch_id = [history.branch_id,reshape(ids,1,[])];
history.event_index = [history.event_index,repmat(k,1,numel(ids))];
end
