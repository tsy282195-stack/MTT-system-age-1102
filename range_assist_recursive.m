function varargout=range_assist_recursive(action,varargin)
%RANGE_ASSIST_RECURSIVE Causal CV range bank from accepted RAW AE/platform.
% Uncertainty is conditional on motion/initial-range priors, not a validated
% independent ranging sensor. Candidate operations never commit observations.
switch action
 case 'update'
  [varargout{1:nargout}]=commit_observation(varargin{:});
 case 'predict'
  varargout{1}=predict_bank(varargin{:});
 case 'candidate'
  [varargout{1:nargout}]=candidate_update(varargin{:});
 otherwise
  error('RangeAssist:UnknownAction','Unknown recursive action %s.',action);
end
end

function [s,report]=commit_observation(s,id,birth,t,z,sensor,o,R)
report=[];
if ~o.enabled,return;end
if isempty(s)||~isfield(s,'format_version')||s.format_version~=2|| ...
        ~isequal(s.identity,[id,birth])
 s=range_assist_empty(id,birth);
end
if any(~isfinite([t;z(:);sensor(:);R(:)]))||numel(z)~=2||numel(sensor)~=3|| ...
        ~isequal(size(R),[2,2])||abs(z(2))>90
 s.status='INVALID_SAMPLE';return;
end
if isfinite(s.last_sample_t)&&t<=s.last_sample_t+1e-9,return;end
if isfinite(s.last_sample_t)&&t-s.last_sample_t>o.gap_reset_s
 first=s.confirmed_t;s=range_assist_empty(id,birth);s.confirmed_t=first;
 s.status='WARMUP_AFTER_GAP';
end
if ~isfinite(s.confirmed_t),s.confirmed_t=t;end
s.last_sample_t=t;
sample=[t;z(:);sensor(:)];noise=[R(1,1);R(1,2);R(2,2)];
if ~s.available&&isempty(s.pending_history)&&~isempty(s.history)
 dz=bearing_delta(z,s.last_raw_ae);
 if any(abs(dz)>[o.jump_az_deg;o.jump_el_deg])
  s.history=zeros(6,0);s.history_noise=zeros(3,0);s.last_attempt_t=NaN;
  s.status='WARMUP_AFTER_JUMP';
 end
end
if ~isempty(s.pending_history)
 dz=bearing_delta(z,s.pending_history(2:3,end));
 if any(abs(dz)>[o.jump_az_deg;o.jump_el_deg])
  s.pending_history=sample;s.pending_noise=noise;
 else
  s.pending_history(:,end+1)=sample;s.pending_noise(:,end+1)=noise;
 end
 if t-s.pending_history(1,1)>=o.min_span_s-1e-9
  s.history=s.pending_history;s.history_noise=s.pending_noise;
  s.pending_history=zeros(6,0);s.pending_noise=zeros(3,0);
  s.available=false;s.models=struct([]);s.weights=[];s.last_attempt_t=NaN;
 else
  s.n_rejected_updates=s.n_rejected_updates+1;s.status='FROZEN_AFTER_JUMP';
  report=state_report(s,t,'frozen');return;
 end
end
if s.available
 prior=predict_bank(s,t,sensor,o);
 [~,d,trial]=candidate_update(prior,z,R,o);
 dz=bearing_delta(z,s.last_raw_ae);
 if any(abs(dz)>[o.jump_az_deg;o.jump_el_deg])|| ...
        ~d.available||strcmp(d.reason,'MODEL_INCONSISTENT')
  s.pending_history=sample;s.pending_noise=noise;
  s.n_rejected_updates=s.n_rejected_updates+1;s.status='FROZEN_AFTER_JUMP';
  report=state_report(s,t,'frozen');return;
 end
 s.models=trial.models;s.weights=trial.weights;s.data_cutoff_s=t;
 s.reference_time_s=t;s.n_recursive_updates=s.n_recursive_updates+1;
 s.status='RECURSIVE_CANDIDATE';
end
if isempty(s.history)||t>s.history(1,end)+1e-9
 s.history(:,end+1)=sample;s.history_noise(:,end+1)=noise;
end
keep=s.history(1,:)>=t-o.window_s-1e-9;
s.history=s.history(:,keep);s.history_noise=s.history_noise(:,keep);
if size(s.history,2)>o.max_samples
 s.history=s.history(:,end-o.max_samples+1:end);
 s.history_noise=s.history_noise(:,end-o.max_samples+1:end);
end
s.last_raw_ae=z(:);s.window_start_s=s.history(1,1);
if ~s.available
 if size(s.history,2)<8||t-s.history(1,1)<o.min_span_s-1e-9,return;end
 if isfinite(s.last_attempt_t)&&t-s.last_attempt_t<o.interval_s-1e-9,return;end
 s.last_attempt_t=t;s.attempts=s.attempts+1;started=tic;
 report=state_report(s,t,'initialize');
 try
  [models,weights,fit]=initialize_bank(s,o);
  s.models=models;s.weights=weights;s.available=true;s.data_cutoff_s=t;
  s.reference_time_s=t;s.initialized_t=t;s.successes=s.successes+1;
  s.solver_exitflag=fit.exitflag;s.angular_cost_rad2=fit.cost;
  s.status='RECURSIVE_CANDIDATE';s.error_id='';s.error_message='';
  report.selected_candidate_index=fit.selected;
  report.candidates=fit.candidates;
 catch ex
  s.failures=s.failures+1;s.status='INITIALIZATION_FAILED';
  s.error_id=ex.identifier;s.error_message=ex.message;
 end
 elapsed=toc(started);
else
 report=state_report(s,t,'update');elapsed=0;
end
if s.available
 summary=summarize(s.models,s.weights,sensor);
 s.range_m=summary.range_m;s.range_std_m=summary.std_m;s.quality=summary.quality;
end
report.status=s.status;report.error_id=s.error_id;report.error_message=s.error_message;
report.range_m=s.range_m;report.range_std_m=s.range_std_m;report.quality=s.quality;
report.initialized_t=s.initialized_t;report.elapsed_s=elapsed;
end

function [models,weights,fit]=initialize_bank(s,o)
range_assist_toolkit(true);
input=make_passive_range_input(s.history(1,:),s.history(2,:),s.history(3,:),s.history(4:6,:),'deg');
options=passive_range_options();options.model='CV';options.apply_smoothing=false;
options.fit.min_span_s=o.min_span_s;options.fit.initial_ranges_m=o.initial_ranges_m;
options.fit.bounds=o.bounds;options.fit.max_iterations=o.max_iterations;
options.fit.max_function_evaluations=o.max_function_evaluations;
transcript=evalc('[result,d]=passive_range_estimate(input,options);'); %#ok<NASGU>
if ~result.candidate_available,error('RangeAssist:NoCandidate','No finite short-window range candidate.');end
models=repmat(struct('x',[],'P',[],'t',s.history(1,end)),1,0);
scores=[];summaries=cell(1,numel(d.fit.candidates));
for k=1:numel(d.fit.candidates)
 c=d.fit.candidates{k};
 summaries{k}=struct('initial_range_m',c.initial_range_m,'cost_rad2',c.cost_rad2, ...
  'exitflag',c.exitflag,'numerical_rank',c.numerical_rank,'condition_number',c.condition_number, ...
  'reference_range_m',NaN,'terminal_range_m',NaN);
 if isempty(c.geometry)||~isfinite(c.cost_rad2),continue;end
 ctx=d.fit.context;L=ctx.length_m;T=ctx.time_scale_s;D=s.history(1,end)-ctx.reference_time_s;
 J=c.jacobian;res=c.geometry.residual_ae_rad(:);Jw=J;rw=res;
 for n=1:size(s.history,2)
  R=[s.history_noise(1,n),s.history_noise(2,n);s.history_noise(2,n),s.history_noise(3,n)]*(pi/180)^2;
  W=chol(spd(R),'lower');ix=2*n-1:2*n;Jw(ix,:)=W\J(ix,:);rw(ix)=W\res(ix);
 end
 % Proper initial-range/velocity priors explicitly bound unobservable modes.
 % They must not be mistaken for information supplied by the angle data.
 pos_std=max(c.initial_range_m*o.initial_position_std_fraction,1000)/L;
 vel_std=o.initial_velocity_std_mps*T/L;
 information=Jw'*Jw+diag(1./[repmat(pos_std^2,1,3),repmat(vel_std^2,1,3)]);
 Pz=spd(information)\eye(6);
 transform=[L*eye(3),L*D/T*eye(3);zeros(3),L/T*eye(3)];
 model=struct('x',[c.geometry.position_ENU_m(:,end);c.geometry.velocity_ENU_mps(:,end)], ...
  'P',spd(transform*Pz*transform'),'t',s.history(1,end));
 models(end+1)=model;scores(end+1)=sum(rw.^2); %#ok<AGROW>
 summaries{k}.reference_range_m=c.geometry.reference_range_m;
 summaries{k}.terminal_range_m=c.geometry.range_m(end);
end
if isempty(models),error('RangeAssist:NoCandidate','No finite model covariance.');end
weights=normalize_weights(exp(-0.5*min(scores-min(scores),100)));
fit=struct('exitflag',result.solver_exitflag,'cost',result.angular_cost_rad2, ...
 'selected',result.selected_candidate_index,'candidates',{summaries});
end

function prior=predict_bank(s,t,sensor,o)
prior=struct('available',false,'reason','NO_CACHE','models',struct([]), ...
 'weights',[],'sensor',sensor,'t',t,'cache_age_s',NaN,'range_m',NaN,'std_m',NaN,'quality',0);
if isempty(s)||~isfield(s,'models')||~s.available||isempty(s.models),return;end
age=t-s.data_cutoff_s;prior.cache_age_s=age;
if age<=0,prior.reason='CACHE_NOT_PRIOR';return;end
if age>o.max_age_s,prior.reason='CACHE_EXPIRED';return;end
if numel(sensor)~=3||any(~isfinite(sensor)),prior.reason='INVALID_GEOMETRY';return;end
prior.models=s.models;prior.weights=s.weights;
for k=1:numel(s.models)
 dt=t-s.models(k).t;
 if dt<0,prior.reason='CACHE_NOT_PRIOR';return;end
 F=[eye(3),dt*eye(3);zeros(3),eye(3)];
 Q=o.process_accel_std_mps2^2*[dt^3/3*eye(3),dt^2/2*eye(3);dt^2/2*eye(3),dt*eye(3)];
 prior.models(k).x=F*s.models(k).x;
 prior.models(k).P=spd(exp(dt/o.window_s)*(F*s.models(k).P*F')+Q);
 prior.models(k).t=t;
end
summary=summarize(prior.models,prior.weights,sensor);
prior.range_m=summary.range_m;prior.std_m=summary.std_m;
prior.quality=summary.quality*exp(-age/o.max_age_s);
prior.available=true;prior.reason='RECURSIVE_CONTINUATION';
end

function [cost,d,trial]=candidate_update(prior,z,R_deg2,o)
cost=NaN;trial=struct('models',struct([]),'weights',[]);
d=struct('available',false,'reason',prior.reason,'reference_range_m',prior.range_m, ...
 'candidate_range_m',NaN,'delta_range_m',NaN,'cache_age_s',prior.cache_age_s, ...
 'range_std_m',prior.std_m,'quality',prior.quality,'range_is_validated',false);
if ~prior.available,return;end
if numel(z)~=2||any(~isfinite(z))||abs(z(2))>90|| ...
 ~isequal(size(R_deg2),[2,2])||any(~isfinite(R_deg2(:)))
 d.reason='INVALID_GEOMETRY';return;
end
try
 R=spd(R_deg2*(pi/180)^2);models=prior.models;
 dr=zeros(size(prior.weights));parts=dr;likelihood=dr;valid=false(size(dr));
 for k=1:numel(models)
  x0=models(k).x;P0=models(k).P;x=x0;
  [h0,H0,g0]=bearing_geometry(x0,prior.sensor);
  nu0=bearing_delta(z,h0)*(pi/180);S0=spd(H0*P0*H0'+R);
  for it=1:o.recursive_iterations
   [h,H]=bearing_geometry(x,prior.sensor);
   nu=bearing_delta(z,h)*(pi/180)+H*(x-x0);
   K=(P0*H')/spd(H*P0*H'+R);next=x0+K*nu;
   if norm(next-x)<1e-4,x=next;break;end
   x=next;
  end
  speed=norm(x(4:6));valid(k)=speed>=o.bounds.speed_min_mps&&speed<=o.bounds.speed_max_mps;
  if ~valid(k),parts(k)=o.cost_cap;likelihood(k)=-50;continue;end
  [~,H,g]=bearing_geometry(x,prior.sensor);
  K=(P0*H')/spd(H*P0*H'+R);M=eye(6)-K*H;
  models(k).x=x;models(k).P=spd(M*P0*M'+K*R*K');
  dr(k)=g.range_m-g0.range_m;
  variance=g0.range_gradient*P0*g0.range_gradient';
  parts(k)=min(dr(k)^2/max(o.scale_m^2+variance,eps),o.cost_cap);
  likelihood(k)=-0.5*(nu0'*(S0\nu0)+2*sum(log(diag(chol(S0)))));
 end
 % An infeasible continuation is an explicit bounded penalty, not free fall-back.
 cost=sum(prior.weights.*parts);d.available=true;
 if ~any(valid)
  d.reason='MODEL_INCONSISTENT';d.candidate_range_m=prior.range_m;
  d.delta_range_m=0;return;
 end
 d.reason='RECURSIVE_CONTINUATION';d.delta_range_m=sum(prior.weights.*dr);
 d.candidate_range_m=prior.range_m+d.delta_range_m;
 weights=prior.weights.*exp(max(likelihood-max(likelihood),-50));
 weights(~valid)=0;weights=normalize_weights(weights);
 trial=struct('models',models,'weights',weights);
catch
 d.available=false;d.reason='OTHER_NUMERICAL_FAILURE';cost=NaN;
end
end

function summary=summarize(models,weights,sensor)
r=zeros(size(weights));v=r;
for k=1:numel(models)
 q=models(k).x(1:3)-sensor(:);r(k)=norm(q);g=[q'/max(r(k),eps),zeros(1,3)];
 v(k)=max(g*models(k).P*g',0);
end
mean_r=sum(weights.*r);spread=sum(weights.*(r-mean_r).^2);
variance=sum(weights.*v)+spread;
relative=sqrt(variance)/max(mean_r,1);dispersion=sqrt(spread)/max(mean_r,1);
quality=1/(1+(relative/0.25)^2)/(1+(dispersion/0.25)^2);
summary=struct('range_m',mean_r,'std_m',sqrt(variance),'quality',quality);
end

function [h,H,g]=bearing_geometry(x,sensor)
q=x(1:3)-sensor(:);xy=hypot(q(1),q(2));r=norm(q);
if xy<=0||r<=0,error('RangeAssist:InvalidGeometry','Undefined bearing.');end
h=[atan2d(q(1),q(2));atan2d(q(3),xy)];
H=[q(2)/xy^2,-q(1)/xy^2,0,0,0,0; ...
 -q(3)*q(1)/(r^2*xy),-q(3)*q(2)/(r^2*xy),xy/r^2,0,0,0];
g=struct('range_m',r,'range_gradient',[q'/r,zeros(1,3)]);
end

function report=state_report(s,t,kind)
report=struct('track_id',s.identity(1),'birth_event',s.identity(2),'time_s',t, ...
 'window_start_s',s.window_start_s,'data_cutoff_s',s.data_cutoff_s, ...
 'n_samples',size(s.history,2),'status',s.status,'error_id','','error_message','', ...
 'selected_candidate_index',[],'candidates',[],'elapsed_s',0, ...
 'range_is_validated',false,'full_fit',[],'kind',kind, ...
 'range_m',s.range_m,'range_std_m',s.range_std_m,'quality',s.quality, ...
 'confirmed_t',s.confirmed_t,'initialized_t',s.initialized_t);
end

function d=bearing_delta(a,b)
d=[mod(a(1)-b(1)+180,360)-180;a(2)-b(2)];
end
function w=normalize_weights(w)
w=reshape(w,1,[]);w=max(w,0);total=sum(w);
if total<=0,w=ones(size(w))/numel(w);else,w=w/total;end
end
function A=spd(A)
A=(A+A')/2;[~,flag]=chol(A);
if flag~=0
 [V,D]=eig(A);floor_value=max(eps(max(abs(diag(D)))),realmin);
 A=V*diag(max(diag(D),floor_value))*V';A=(A+A')/2;
end
end
