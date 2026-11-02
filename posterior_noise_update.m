function [bank, R_next, info] = posterior_noise_update( ...
        bank, R_previous, residual, Pzz_post, innovation, S_prior, p)
%POSTERIOR_NOISE_UPDATE Residual-based Sage-Husa R for the NEXT measurement.
% M uses the pre-robust prior innovation covariance. Normal samples have d=1.
% residual and Pzz_post must use the same measurement space as R_previous.
R_next = R_previous;
info = struct('updated', false, 'M', NaN, 'd', NaN, ...
    'threshold', p.threshold, 'posterior_residual', residual(:));
if ~p.enabled, return; end
m = size(R_previous, 1);
if numel(residual) ~= m || numel(innovation) ~= m || ...
        ~isequal(size(Pzz_post), [m, m]) || ~isequal(size(S_prior), [m, m]) || ...
        any(~isfinite([R_previous(:); residual(:); Pzz_post(:); ...
        innovation(:); S_prior(:)]))
    return;
end
S_prior = (S_prior + S_prior') / 2;
[L, flag] = chol(S_prior, 'lower');
if flag ~= 0, return; end
info.M = sum((L \ innovation(:)).^2);
info.d = 1;
if info.M > p.threshold
    info.d = max(p.d_min, p.threshold / info.M);
end
candidate = residual(:) * residual(:)' + Pzz_post;
R_next = (1 - info.d) * R_previous + info.d * candidate;
R_next = (R_next + R_next') / 2;
R_next = R_next + diag(max(p.min_diag - diag(R_next), 0));
% Preserve full covariance/cross terms; repair only numerical indefiniteness.
[~, flag] = chol(R_next);
if flag ~= 0
    [V, D] = eig(R_next);
    eigen_floor = max(eps(max(abs(diag(D)))), realmin);
    R_next = V * diag(max(diag(D), eigen_floor)) * V';
    R_next = (R_next + R_next') / 2;
    R_next = R_next + diag(max(p.min_diag - diag(R_next), 0));
end
if any(~isfinite(R_next(:)))
    R_next = R_previous;
    return;
end
info.updated = true;
n = 1;
if isstruct(bank) && isfield(bank, p.key)
    n = bank.(p.key).n_updates + 1;
end
bank.(p.key) = struct('R', R_next, 'info', info, 'n_updates', n);
end
