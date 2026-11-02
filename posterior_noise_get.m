function R = posterior_noise_get(bank, R_nominal, p)
%POSTERIOR_NOISE_GET Read the last accepted estimate for this sensor/space.
% Disabled adaptation returns the supplied nominal R even if a bank exists.
R = R_nominal;
if p.enabled && isstruct(bank) && isfield(bank, p.key)
    saved = bank.(p.key).R;
    if isequal(size(saved), size(R_nominal)) && all(isfinite(saved(:)))
        R = saved;
    end
end
end
