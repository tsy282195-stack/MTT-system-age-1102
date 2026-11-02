function c=angular_motion_context(input,model,scales)
%ANGLE_MOTION_CONTEXT Strict angle/platform-only OFFLINE diagnostic interface.
fields(input,{'schema_version','role','coordinate_frame','angle_units','time_s','azimuth_rad','elevation_rad','platform_position_ENU_m'});
fields(scales,{'length_m','time_s'});
if ~strcmp(input.schema_version,'angular-motion-input-v1')||~strcmp(input.role,'ANGLE_PLATFORM_ONLY')||~strcmp(input.coordinate_frame,'local_enu')||~strcmp(input.angle_units,'rad')
 error('BO3D:Angular:InvalidSchema','Angle/platform-only local ENU radians required.');
end
if ~ischar(model)||~ismember(model,{'CV','CA'}),error('BO3D:Angular:InvalidModel','Explicit CV or CA required.');end
for f={'length_m','time_s'}
 x=scales.(f{1});if ~isa(x,'double')||~isreal(x)||~isscalar(x)||~isfinite(x)||x<=0,error('BO3D:Angular:InvalidScale','Positive finite double scales required.');end
end
t=input.time_s;
if ~isa(t,'double')||~isreal(t)||~isvector(t)||isempty(t)||any(~isfinite(t))||any(diff(t)<=0),error('BO3D:Angular:InvalidTime','Strictly increasing finite timestamps required.');end
t=t(:).';n=numel(t);az=input.azimuth_rad;el=input.elevation_rad;p=input.platform_position_ENU_m;
if ~isa(az,'double')||~isa(el,'double')||~isreal(az)||~isreal(el)||~isvector(az)||~isvector(el)||numel(az)~=n||numel(el)~=n||any(~isfinite(az))||any(~isfinite(el))||any(abs(el)>pi/2)
 error('BO3D:Angular:InvalidAngles','Finite angle vectors and physical elevations required.');
end
if ~isa(p,'double')||~isreal(p)||~isequal(size(p),[3,n])||any(~isfinite(p),'all'),error('BO3D:Angular:InvalidPlatform','Finite known 3-by-N positions required.');end
tau=t-t(1);s=tau/scales.time_s;
c=struct('model',model,'dimension',6+3*strcmp(model,'CA'),'time_s',t,'reference_time_s',t(1),'tau_s',tau,'scaled_time',s, ...
 'length_m',scales.length_m,'time_scale_s',scales.time_s,'origin_ENU_m',p(:,1),'platform_ENU_m',p, ...
 'azimuth_rad',az(:).','elevation_rad',el(:).');
end
function fields(x,expected)
if ~isstruct(x)||~isscalar(x),error('BO3D:Angular:InvalidSchema','Scalar struct required.');end
if any(~ismember(fieldnames(x),expected)),error('BO3D:Angular:UnknownField','Unknown fields, including truth/prior fields, are prohibited.');end
if ~all(ismember(expected,fieldnames(x))),error('BO3D:Angular:MissingField','Required field missing.');end
end
