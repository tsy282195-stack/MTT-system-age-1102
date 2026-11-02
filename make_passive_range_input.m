function input=make_passive_range_input(time_s,azimuth,elevation,platform_position_ENU_m,angle_unit)
% Single associated target; aligned absolute platform positions, fixed ENU.
% Azimuth north-zero clockwise; elevation horizontal-zero upward.
if ~ischar(angle_unit)||~ismember(angle_unit,{'rad','deg','mrad'})
 error('PassiveToolkit:InvalidAngleUnit','Explicit char rad, deg or mrad required.');
end
if ~isa(azimuth,'double')||~isa(elevation,'double')||~isreal(azimuth)||~isreal(elevation)
 error('PassiveToolkit:InvalidAngles','Real double angle arrays required.');
end
factor=1;if strcmp(angle_unit,'deg'),factor=pi/180;elseif strcmp(angle_unit,'mrad'),factor=1/1000;end
input=struct('schema_version','angular-motion-input-v1','role','ANGLE_PLATFORM_ONLY', ...
 'coordinate_frame','local_enu','angle_units','rad','time_s',time_s, ...
 'azimuth_rad',azimuth*factor,'elevation_rad',elevation*factor,'platform_position_ENU_m',platform_position_ENU_m);
c=angular_motion_context(input,'CV',struct('length_m',1,'time_s',1));
input.time_s=c.time_s;input.azimuth_rad=c.azimuth_rad;input.elevation_rad=c.elevation_rad;
end
