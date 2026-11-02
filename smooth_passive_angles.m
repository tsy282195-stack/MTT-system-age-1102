function [smoothed,d]=smooth_passive_angles(input,cfg)
%SMOOTH_PASSIVE_ANGLES One offline local quadratic pass on irregular AE times.
% Preserve every timestamp/platform state; no padding/outlier/jump correction.
fields={'method','polynomial_degree','half_window_s','minimum_points','maximum_gap_s','passes','is_causal'};
if ~isstruct(cfg)||~isscalar(cfg)||~isequal(sort(fieldnames(cfg)),sort(fields(:)))||~strcmp(cfg.method,'LOCAL_QUADRATIC_PHYSICAL_TIME')||~isequal(cfg.polynomial_degree,2)||~isequal(cfg.passes,1)||~islogical(cfg.is_causal)||~isscalar(cfg.is_causal)||cfg.is_causal
 error('BO3D:Smoothing:InvalidConfig','Exact explicit one-pass offline quadratic configuration required.');
end
for f={'half_window_s','minimum_points','maximum_gap_s'}
 x=cfg.(f{1});if ~isa(x,'double')||~isreal(x)||~isscalar(x)||~isfinite(x)||x<=0,error('BO3D:Smoothing:InvalidConfig','Finite positive scalar settings required.');end
end
if fix(cfg.minimum_points)~=cfg.minimum_points||cfg.minimum_points<3,error('BO3D:Smoothing:InvalidConfig','At least three support points required.');end
% Reuse unchanged strict angle/platform-only schema validation, no truth fields.
angular_motion_context(input,'CV',struct('length_m',1,'time_s',1));
t=input.time_s(:).';n=numel(t);az=input.azimuth_rad(:).';el=input.elevation_rad(:).';
segment=cumsum([1,diff(t)>cfg.maximum_gap_s]);unwrapped=zeros(1,n);out=zeros(2,n);ranges=zeros(2,n);counts=zeros(1,n);ranks=zeros(1,n);thresholds=zeros(1,n);rss=zeros(2,n);
for sid=1:segment(end)
 ix=find(segment==sid);unwrapped(ix)=unwrap(az(ix));
end
for i=1:n
 use=find(segment==segment(i)&abs(t-t(i))<=cfg.half_window_s);
 if numel(use)<cfg.minimum_points,error('BO3D:Smoothing:InsufficientSupport','Frame %d has %d support samples, requires %d; no fallback or padding.',i,numel(use),cfg.minimum_points);end
 x=(t(use)-t(i))'/cfg.half_window_s;X=[ones(numel(use),1),x,x.^2];Y=[unwrapped(use)',el(use)'];
 [Q,R]=qr(X,0);threshold=max(size(X))*eps(norm(R,2));rankX=sum(abs(diag(R))>threshold);
 if rankX~=3,error('BO3D:Smoothing:RankDeficientWindow','Local design at frame %d has rank %d; no repair.',i,rankX);end
 b=R\(Q'*Y);out(:,i)=b(1,:)';ranges(:,i)=[use(1);use(end)];counts(i)=numel(use);ranks(i)=rankX;thresholds(i)=threshold;rss(:,i)=sum((Y-X*b).^2,1)';
end
if any(~isfinite(out),'all'),error('BO3D:Smoothing:NonfiniteOutput','Nonfinite fit; no substitution.');end
if any(abs(out(2,:))>pi/2),error('BO3D:Smoothing:OutputElevationDomain','Smoothed elevation outside physical domain; no clipping.');end
smoothed=input;smoothed.azimuth_rad=atan2(sin(out(1,:)),cos(out(1,:)));smoothed.elevation_rad=out(2,:);
d=struct('method',cfg.method,'passes',1,'polynomial_degree',2,'is_causal',false,'uses_future_within_selected_prefix',true, ...
 'half_window_s',cfg.half_window_s,'minimum_points',cfg.minimum_points,'maximum_gap_s',cfg.maximum_gap_s,'segment_id',segment, ...
 'support_indices_first_last',ranges,'support_count',counts,'local_design_rank',ranks,'local_rank_threshold',thresholds,'local_residual_sum_squares_rad2',rss, ...
 'azimuth_unwrapped_input_rad',unwrapped,'azimuth_unwrapped_smoothed_rad',out(1,:), ...
 'change_az_rad',atan2(sin(smoothed.azimuth_rad-az),cos(smoothed.azimuth_rad-az)),'change_el_rad',out(2,:)-el, ...
 'boundary_policy','TRUNCATE_WINDOW_NO_PADDING_NO_EXTRAPOLATION','gap_policy','SPLIT_AT_CONFIGURED_GAP_NO_BRIDGING', ...
 'statistical_note','SMOOTHED_SAMPLES_ARE_CORRELATED_NO_INDEPENDENT_NOISE_OR_INFORMATION_GAIN_CLAIM');
end
