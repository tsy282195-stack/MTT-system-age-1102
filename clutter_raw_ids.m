function ids = clutter_raw_ids(cfg)
%CLUTTER_RAW_IDS Evaluation-only clutter IDs; [] disables the convention.
% A missing field keeps the historical raw-ID-zero definition. An explicit
% empty field must not fall back to zero.
ids = 0;
if nargin >= 1 && isfield(cfg, 'clutter_raw_ids')
    ids = cfg.clutter_raw_ids;
end
if ~isnumeric(ids) || ~isreal(ids) || ...
        (~isempty(ids) && ~isvector(ids)) || ...
        any(~isfinite(ids(:))) || any(ids(:) ~= round(ids(:)))
    error('clutter_raw_ids:InvalidConfig', ...
        'cfg.clutter_raw_ids 必须是有限整数编号向量，或 []（关闭按编号定义杂波）。');
end
ids = unique(reshape(double(ids), 1, []));
end
