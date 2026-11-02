function [res,J,g]=angular_motion_residual(z,c)
%ANGULAR_MOTION_RESIDUAL Equal-weight AE residuals in radians, NOT metre projection.
% z=[(p_ref-platform_ref)/L; v*T/L; a*T^2/L (CA only)].
% No distance floor, regularizer, normalization of observed data, or truth.
if ~isa(z,'double')||~isreal(z)||numel(z)~=c.dimension||any(~isfinite(z)),error('BO3D:Angular:InvalidParameter','Finite real parameter vector required.');end
z=z(:);s=c.scaled_time;L=c.length_m;T=c.time_scale_s;n=numel(s);
pt=c.origin_ENU_m+L*(z(1:3)+z(4:6)*s);
vel=repmat(z(4:6)*L/T,1,n);acc=zeros(3,1);
if c.dimension==9,pt=pt+.5*L*z(7:9)*(s.^2);vel=vel+L/T*z(7:9)*s;acc=z(7:9)*L/T^2;end
q=pt-c.platform_ENU_m;h=hypot(q(1,:),q(2,:));r=hypot(h,q(3,:));
if any(~isfinite(q),'all')||any(~isfinite(r))||any(r==0)||any(h==0),error('BO3D:Angular:UndefinedGeometry','Zero range, azimuth pole or nonfinite geometry; no numerical floor is applied.');end
az=atan2(q(1,:),q(2,:));el=atan2(q(3,:),h);da=az-c.azimuth_rad;
ae=[atan2(sin(da),cos(da));el-c.elevation_rad];res=ae(:);
if nargout>1
 J=zeros(2*n,c.dimension);
 for k=1:n
  dp=L*[eye(3),s(k)*eye(3)];if c.dimension==9,dp=[dp,.5*L*s(k)^2*eye(3)];end %#ok<AGROW>
  gradAz=[q(2,k)/h(k),-q(1,k)/h(k),0]/h(k);
  gradEl=[-q(3,k)/r(k)*q(1,k)/h(k),-q(3,k)/r(k)*q(2,k)/h(k),h(k)/r(k)]/r(k);
  J(2*k-1:2*k,:)=[gradAz;gradEl]*dp;
 end
 if any(~isfinite(J),'all'),error('BO3D:Angular:UndefinedGeometry','Nonfinite analytic Jacobian.');end
end
if nargout>2
 g=struct('position_ENU_m',pt,'velocity_ENU_mps',vel,'acceleration_ENU_mps2',acc,'relative_position_ENU_m',q, ...
  'range_m',r,'azimuth_pred_rad',az,'elevation_pred_rad',el,'residual_ae_rad',ae,'reference_range_m',r(1));
end
end
