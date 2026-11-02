function d=motion_kinematic_constraints(v0,a,duration,bounds)
%MOTION_KINEMATIC_CONSTRAINTS Exact CV/CA continuous-window norm extrema.
% Bounds are absolute target speed and acceleration, NOT ENU components.
if ~isstruct(bounds)||~isscalar(bounds)||~isequal(sort(fieldnames(bounds)),sort({'speed_min_mps';'speed_max_mps';'acceleration_max_mps2'}))
 error('BO3D:Constrained:InvalidBounds','Exact three-field bounds struct required.');
end
b=[bounds.speed_min_mps,bounds.speed_max_mps,bounds.acceleration_max_mps2];
if ~isa(b,'double')||~isreal(b)||numel(b)~=3||any(~isfinite(b))||b(1)<0||b(2)<=b(1)||b(3)<0
 error('BO3D:Constrained:InvalidBounds','Finite ordered speed bounds and nonnegative acceleration bound required.');
end
if ~isa(v0,'double')||~isa(a,'double')||~isreal(v0)||~isreal(a)||numel(v0)~=3||numel(a)~=3||any(~isfinite(v0(:)))||any(~isfinite(a(:)))
 error('BO3D:Constrained:InvalidKinematics','Finite real three-vectors required.');
end
if ~isa(duration,'double')||~isreal(duration)||~isscalar(duration)||~isfinite(duration)||duration<0,error('BO3D:Constrained:InvalidDuration','Finite nonnegative duration required.');end
v0=v0(:);a=a(:);an=hypot(hypot(a(1),a(2)),a(3));vend=v0+a*duration;
norm3=@(v)hypot(hypot(v(1),v(2)),v(3));s0=norm3(v0);s1=norm3(vend);
if an==0,rawCritical=0;else,rawCritical=-sum(v0.*(a/an))/an;end
critical=min(duration,max(0,rawCritical));vcrit=v0+a*critical;smin=norm3(vcrit);smax=max(s0,s1);
if any(~isfinite([an;s0;s1;critical;smin;smax]))||any(~isfinite(vend))
 error('BO3D:Constrained:NonfiniteKinematics','Nonfinite derived kinematics; no repair.');
end
lower=smin<b(1);upper=smax>b(2);accel=an>b(3);code=double(lower)+2*double(upper)+4*double(accel);
d=struct('feasible',code==0,'violation_code',code,'speed_min_mps',smin,'speed_max_mps',smax,'speed_at_start_mps',s0,'speed_at_end_mps',s1, ...
 'speed_min_time_from_reference_s',critical,'unconstrained_critical_time_s',rawCritical,'acceleration_norm_mps2',an, ...
 'margins',[smin-b(1),b(2)-smax,b(3)-an],'interpretation','CONTINUOUS_WINDOW_ABSOLUTE_TARGET_NORMS','comparison_tolerance',0);
end
