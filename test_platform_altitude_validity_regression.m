function result = test_platform_altitude_validity_regression()
%TEST_PLATFORM_ALTITUDE_VALIDITY_REGRESSION Check numeric/Chinese altitude flags.

filepath = [tempname, '.csv'];
cleanup = onCleanup(@() delete_if_present(filepath)); %#ok<NASGU>
fid = fopen(filepath, 'w');
assert(fid >= 0, 'Unable to create temporary platform file.');
fprintf(fid, 'time,lat,lon,alt,alt_valid\n');
fprintf(fid, '0,30,120,100,有效\n');
fprintf(fid, '1,30,120,999,无效\n');
fprintf(fid, '2,30,120,300,1\n');
fclose(fid);

cfg = struct();
cfg.platform = struct('time_col', 1, 'lat_col', 2, 'lon_col', 3, ...
    'alt_col', 4, 'alt_valid_col', 5, 'angle_unit', 'deg', ...
    'time_format', 'seconds');
cfg.delimiter = 'auto';
cfg.read_start_percent = 0; cfg.read_end_percent = 100;
cfg.read_percent = 100; cfg.max_rows = inf; cfg.time_range_s = [];
cfg.platform_max_extrapolation_s = 0;
cfg.platform_alt_max_interp_gap_s = 3;

platform = load_platform_txt(filepath, cfg);
assert(isequal(platform.alt_valid(:).', [true, false, true]), ...
    'Altitude validity flags were not retained.');
assert(max(abs(platform.alt_m(:).' - [100, 200, 300])) < 1e-12, ...
    'Invalid altitude was not interpolated from valid neighbors.');
assert(platform.n_altitude_interpolated == 1, ...
    'Interpolated altitude count is incorrect.');

result = struct('passed', true, 'altitude_m', platform.alt_m(:).', ...
    'validity', platform.alt_valid(:).');
fprintf('平台高度有效性回归通过：无效高度已由100/300m插值为200m。\n');
end

function delete_if_present(filepath)
if exist(filepath, 'file') == 2, delete(filepath); end
end
