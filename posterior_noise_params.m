function p = posterior_noise_params(cfg, source, space)
%POSTERIOR_NOISE_PARAMS Independent active/passive posterior-R settings.
% source: 'active' or 'passive'; space: 'xyz' [m] or 'angle' [deg].
prefix = [source, '_adapt_R_'];
p.enabled = setting(cfg, [prefix, 'enabled'], false);
confidence_default = 0.99; d_default = 0.01; angle_floor = [0.01, 0.01];
if strcmp(source, 'passive')
    confidence_default = 0.995; d_default = 0.005; angle_floor = [0.005, 0.005];
end
p.confidence = setting(cfg, [prefix, 'confidence'], confidence_default);
p.d_min = setting(cfg, [prefix, 'd_min'], d_default);
if strcmp(space, 'xyz')
    p.key = [source, '_xyz'];
    min_std = setting(cfg, [prefix, 'min_std_xyz_m'], [100, 100, 100]);
    m = 3;
else
    p.key = [source, '_ae'];
    min_std = setting(cfg, [prefix, 'min_std_angle_deg'], angle_floor);
    m = 2;
end
if ~isscalar(p.enabled) || ~(islogical(p.enabled) || isnumeric(p.enabled)) || ...
        ~isreal(p.enabled) || ~isfinite(p.enabled) || ~ismember(p.enabled, [0, 1])
    error('posterior_noise:InvalidFlag', '%senabled must be true or false.', prefix);
end
p.enabled = logical(p.enabled);
if ~isnumeric(p.confidence) || ~isreal(p.confidence) || ...
        ~isscalar(p.confidence) || ~isfinite(p.confidence) || ...
        p.confidence <= 0 || p.confidence >= 1
    error('posterior_noise:InvalidConfidence', '%sconfidence must be in (0,1).', prefix);
end
if ~isnumeric(p.d_min) || ~isreal(p.d_min) || ~isscalar(p.d_min) || ...
        ~isfinite(p.d_min) || p.d_min <= 0 || p.d_min > 1
    error('posterior_noise:InvalidWeight', '%sd_min must be in (0,1].', prefix);
end
if ~isnumeric(min_std) || ~isreal(min_std) || numel(min_std) ~= m || ...
        any(~isfinite(min_std(:))) || any(min_std(:) <= 0)
    error('posterior_noise:InvalidFloor', '%s minimum standard deviations are invalid.', prefix);
end
p.min_diag = min_std(:).^2;
% chi2inv(probability,m), without requiring Statistics Toolbox.
p.threshold = chi_square_threshold(p.confidence, m);
end

function value = setting(cfg, name, fallback)
if isfield(cfg, name) && ~isempty(cfg.(name))
    value = cfg.(name);
else
    value = fallback;
end
end

function c = chi_square_threshold(probability, m)
persistent probabilities dimensions thresholds
idx = find(probabilities == probability & dimensions == m, 1);
if isempty(idx)
    c = 2 * gammaincinv(probability, m / 2);
    probabilities(end + 1) = probability;
    dimensions(end + 1) = m;
    thresholds(end + 1) = c;
else
    c = thresholds(idx);
end
end
