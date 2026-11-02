function [cost,d]=range_assist_batch_pair_cost(s,t,sensor,predicted_ae_deg,virtual_ae_deg,o)
% Local continuation sensitivity, NOT a measured range or a new full fit.
% Adding the endpoint to the frozen historical QR system gives dz/d(AE).
cost=NaN;d=struct('available',false,'reason','NO_CACHE','reference_range_m',NaN, ...
 'candidate_range_m',NaN,'delta_range_m',NaN,'cache_age_s',NaN,'range_is_validated',false);
if isempty(s)||~s.available,return;end
d.cache_age_s=t-s.data_cutoff_s;
if d.cache_age_s<=0,d.reason='CACHE_NOT_PRIOR';return;end
if d.cache_age_s>o.max_age_s,d.reason='CACHE_EXPIRED';return;end
if any(~isfinite([sensor(:);predicted_ae_deg(:);virtual_ae_deg(:)]))|| ...
        abs(predicted_ae_deg(2))>90||abs(virtual_ae_deg(2))>90
 d.reason='INVALID_GEOMETRY';return;end
try
 n=numel(s.z);dt=t-s.reference_time_s;st=dt/s.time_scale_s;L=s.length_m;
 B=L*[eye(3),st*eye(3)];if n==9,B=[B,.5*L*st^2*eye(3)];end
 q=s.origin_ENU_m+B*s.z-sensor(:);h=hypot(q(1),q(2));r=norm(q);
 if r==0||h==0,error('RangeAssist:UndefinedGeometry','Zero range or azimuth pole.');end
 H=[q(2)/h^2,-q(1)/h^2,0; -q(3)*q(1)/(r^2*h),-q(3)*q(2)/(r^2*h),h/r^2]*B;
 da=virtual_ae_deg(1)-predicted_ae_deg(1);
 dy=[atan2(sind(da),cosd(da));deg2rad(virtual_ae_deg(2)-predicted_ae_deg(2))];
 [U,S,V]=svd([s.history_R;H],0);v=diag(S);
 keep=v>max(1e-10*v(1),max(size(U))*eps(v(1)));
 if ~any(keep),error('RangeAssist:NoSensitivity','No finite continuation direction.');end
 rhs=[zeros(size(s.history_R,1),1);dy];
 dz=V(:,keep)*((U(:,keep)'*rhs)./v(keep));z=s.z+dz;
 vel=z(4:6)*L/s.time_scale_s;acc=zeros(3,1);
 if n==9,acc=z(7:9)*L/s.time_scale_s^2;end
 chk=motion_kinematic_constraints(vel,acc,dt,o.bounds);
 if ~chk.feasible,d.reason='CONTINUATION_INFEASIBLE';return;end
 rr=norm(q+B*dz);
 if ~isfinite(rr)||rr<=0,error('RangeAssist:InvalidContinuation','Nonpositive/nonfinite range.');end
 d.reference_range_m=r;d.candidate_range_m=rr;d.delta_range_m=rr-r;
 cost=min((d.delta_range_m/o.scale_m)^2,1);
 d.available=true;d.reason='LOCAL_LINEAR_CONTINUATION';
catch ex
 d.reason=ex.identifier;
end
end
