function result = test_joint_2d_association_regression()
%TEST_JOINT_2D_ASSOCIATION_REGRESSION Focused coupled-bearing regressions.
% Run this function in MATLAB after changes to 2-D association or
% measurement ownership.

predicted_b = [75; -0.9];
measurement_a = [-100; -0.9];
measurement_b = [75; -0.85];
residual_a = joint_bearing_residual(measurement_a, predicted_b);
residual_b = joint_bearing_residual(measurement_b, predicted_b);
assert(abs(residual_a(1)) > 15 && abs(residual_a(2)) < 1e-12, ...
    'A must be rejected by azimuth even when elevation is indistinguishable.');
assert(abs(residual_b(1)) < 1e-12 && abs(residual_b(2)) < 0.1, ...
    'B must remain a valid coupled azimuth/elevation candidate.');

S = diag([0.5^2, 0.2^2]);
nis_a = residual_a' * (S \ residual_a);
nis_b = residual_b' * (S \ residual_b);
assert(nis_a > 9.2103 && nis_b < 9.2103, ...
    'Joint NIS must distinguish A from B despite nearly equal elevation.');

% Elevation favors the crossed pairing, while azimuth continuity favors the
% diagonal pairing. Equal weighting crosses; a 4:1 az/el ranking keeps IDs.
pred = [0, 0.2; -0.125, 0.125];
meas = [0.02, 0.18; 0.125, -0.125];
sigma = [0.05; 0.05];
az_cost = zeros(2); el_cost = zeros(2);
for i = 1:2
    for j = 1:2
        nu = joint_bearing_residual(meas(:, j), pred(:, i));
        az_cost(i, j) = (nu(1) / sigma(1)) ^ 2;
        el_cost(i, j) = (nu(2) / sigma(2)) ^ 2;
    end
end
equal_pairs = solve_global_assignment(az_cost + el_cost, 1000);
waz = 2 * 4 / 5; wel = 2 * 1 / 5;
weighted_pairs = solve_global_assignment(waz * az_cost + wel * el_cost, 1000);
assert(isequal(equal_pairs, [1, 2; 2, 1]), ...
    'Synthetic equal-weight case must expose the crossed optimum.');
assert(isequal(weighted_pairs, [1, 1; 2, 2]), ...
    'Azimuth-priority ranking must preserve the continuous pairing.');

wrap_residual = joint_bearing_residual([-179.9; -1], [179.9; -1]);
assert(abs(wrap_residual(1) - 0.2) < 1e-9, ...
    'Azimuth association must remain continuous across +/-180 degrees.');
plot_az = azimuth_for_plot([-179.9, 179.9], 180);
assert(max(abs(plot_az - [180.1, 179.9])) < 1e-9, ...
    'The 180-degree-centered plot must draw +/-180 together.');

a = struct('id', [1001, 1002], ...
    'type', {{'passive', 'passive'}}, 'meas_index', [1, 1], ...
    'filter_dim', [2, 2], 'input_dim', [2, 2], ...
    'range_updated', [false, false]);
duplicate_detected = false;
try
    joint_measurement_disposition(1, a, 'passive', false(1, 1), ...
        1, {'suppressed_unassociated'});
catch ME
    duplicate_detected = strcmp(ME.identifier, ...
        'run_filter_joint_2d3d:MeasurementDispositionMismatch');
end
assert(duplicate_detected, ...
    'Duplicate consumption of one passive measurement must be rejected.');

companion = struct('id', 1001, 'type', {{'companion_passive_2d'}}, ...
    'meas_index', 1, 'filter_dim', 2, 'input_dim', 2, ...
    'range_updated', false);
ledger = joint_measurement_disposition(1, companion, 'passive', false(1, 1), ...
    2, {'suppressed_unassociated'});
assert(ledger.action(1) == 1 && ledger.track_id(1) == 1001, ...
    'Detached companion association must be recorded as passive-associated.');

result = struct('passed', true, 'nis_a', nis_a, 'nis_b', nis_b, ...
    'azimuth_separation_deg', abs(residual_a(1)), ...
    'equal_weight_pairs', equal_pairs, 'weighted_pairs', weighted_pairs, ...
    'duplicate_consumption_detected', duplicate_detected);
fprintf(['二维联合关联回归通过: A/B方位差=%.1fdeg, ' ...
    'NIS(A)=%.3f, NIS(B)=%.3f。\n'], ...
    result.azimuth_separation_deg, nis_a, nis_b);
end
