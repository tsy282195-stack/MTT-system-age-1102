function fit=fit_constrained_angular_motion(input,model,cfg)
% New isolated constrained diagnostic; accepted CV/CA angle kernels unchanged.
expected={'method','solver','scales','initial_ranges_m','bounds','max_iterations','max_function_evaluations','tol_x','tol_fun','min_measurements','min_span_s','rank_relative_tolerance','near_cost_relative','near_cost_absolute_rad2','range_spread_relative','initialization','prior_source'};
if ~isstruct(cfg)||~isscalar(cfg)||~isequal(sort(fieldnames(cfg)),sort(expected(:)))||~strcmp(cfg.method,'OFFLINE_HARD_KINEMATIC_DIAGNOSTIC')||~strcmp(cfg.solver,'fminsearch_inf_feasibility_domain')||~strcmp(cfg.initialization,'FIXED_RANGE_SVD_DIRECTIONS_INTERIOR_SPEED_ACCELERATION')
 error('BO3D:Constrained:InvalidConfig','Exact explicitly configured constrained diagnostic required.');
end
for f={'max_iterations','max_function_evaluations','tol_x','tol_fun','min_measurements','min_span_s','rank_relative_tolerance','near_cost_relative','near_cost_absolute_rad2','range_spread_relative'}
 x=cfg.(f{1});if ~isa(x,'double')||~isreal(x)||~isscalar(x)||~isfinite(x)||x<=0,error('BO3D:Constrained:InvalidConfig','Finite positive numerical settings required.');end
end
if any(fix([cfg.max_iterations,cfg.max_function_evaluations,cfg.min_measurements])~=[cfg.max_iterations,cfg.max_function_evaluations,cfg.min_measurements])||cfg.rank_relative_tolerance>=1,error('BO3D:Constrained:InvalidConfig','Invalid budgets/rank tolerance.');end
c=angular_motion_context(input,model,cfg.scales);n=numel(c.time_s);D=c.tau_s(end);L=c.length_m;T=c.time_scale_s;
if n<cfg.min_measurements||D<cfg.min_span_s,error('BO3D:Constrained:InsufficientSpan','Insufficient valid sample count/span.');end
r0=cfg.initial_ranges_m;if ~isa(r0,'double')||~isreal(r0)||~isvector(r0)||isempty(r0)||any(~isfinite(r0))||any(r0<=0),error('BO3D:Constrained:InvalidConfig','Positive finite initial distances required.');end
% Validate bounds separately; choose an explicitly interior seed speed.
motion_kinematic_constraints(zeros(3,1),zeros(3,1),D,cfg.bounds);
vmid=(cfg.bounds.speed_min_mps+cfg.bounds.speed_max_mps)/2;
aseed=min(cfg.bounds.acceleration_max_mps2/4,(cfg.bounds.speed_max_mps-cfg.bounds.speed_min_mps)/(4*D));
u=[cos(c.elevation_rad).*sin(c.azimuth_rad);cos(c.elevation_rad).*cos(c.azimuth_rad);sin(c.elevation_rad)];
A=zeros(3*n,c.dimension-3);rhs=zeros(3*n,1);Ps=zeros(3*n,3);
for i=1:n
 P=eye(3)-u(:,i)*u(:,i)';ix=3*i-2:3*i;Ps(ix,:)=P;A(ix,1:3)=c.scaled_time(i)*P;
 if c.dimension==9,A(ix,4:6)=.5*c.scaled_time(i)^2*P;end
 rhs(ix)=P*(c.platform_ENU_m(:,i)-c.origin_ENU_m)/L;
end
[Us,Ss,Vs]=svd(A,0);ss=diag(Ss);seedThreshold=max(size(A))*eps(ss(1));keep=ss>seedThreshold;
candidates=cell(1,numel(r0));costs=NaN(size(r0));ranges=costs;trace=[];traceCount=0;evals=[];evalCount=0;
for k=1:numel(r0)
 zp=r0(k)*u(:,1)/L;seed=Vs(:,keep)*((Us(:,keep)'*(rhs-Ps*zp))./ss(keep));vs=seed(1:3)*L/T;ns=norm3(vs);
 if ns==0,vdir=[0;1;0];else,vdir=vs/ns;end
 vinit=vmid*vdir;ainit=zeros(3,1);
 if c.dimension==9
  as=seed(4:6)*L/T^2;na=norm3(as);if na>0,ainit=aseed*as/na;end
 end
 z0=[zp;vinit*T/L];if c.dimension==9,z0=[z0;ainit*T^2/L];end %#ok<AGROW>
 initCheck=motion_kinematic_constraints(vinit,ainit,D,cfg.bounds);assert(initCheck.feasible,'BO3D:Constrained:InfeasibleSeed','Constructed seed must be interior feasible.');
 x=struct('initial_range_m',r0(k),'z0',z0,'initial_constraints',initCheck,'conditional_linear_seed',seed,'initialization_singular_values',ss,'initialization_threshold',seedThreshold, ...
  'z',[],'cost_rad2',NaN,'exitflag',NaN,'solver_output',[],'error_id','','error_message','','error_stack',[],'iteration_trace',[],'objective_evaluations',[], ...
  'geometry',[],'jacobian',[],'singular_values',[],'right_vectors',[],'rank_threshold',NaN,'numerical_rank',NaN,'condition_number',NaN,'constraints',[],'range_is_validated',false);
 trace=zeros(cfg.max_iterations+5,3+c.dimension);traceCount=0;evals=zeros(cfg.max_function_evaluations+2*c.dimension+10,6+c.dimension);evalCount=0;
 try
  opts=optimset('Display','off','MaxIter',cfg.max_iterations,'MaxFunEvals',cfg.max_function_evaluations,'TolX',cfg.tol_x,'TolFun',cfg.tol_fun,'OutputFcn',@capture);
  [z,fval,ef,output]=fminsearch(@objective,z0,opts);chk=check(z);
  if ~chk.feasible||~isfinite(fval),error('BO3D:Constrained:InvalidTerminal','Terminal candidate must be feasible with finite objective.');end
  [r,J,g]=angular_motion_residual(z,c);[~,Sj,Vj]=svd(J,0);sj=diag(Sj);tau=max(cfg.rank_relative_tolerance*sj(1),max(size(J))*eps(sj(1)));rankJ=sum(sj>tau);
  x.z=z;x.cost_rad2=fval;x.exitflag=ef;x.solver_output=output;x.geometry=g;x.jacobian=J;x.singular_values=sj;x.right_vectors=Vj;x.rank_threshold=tau;x.numerical_rank=rankJ;x.constraints=chk;
  if rankJ==c.dimension,x.condition_number=sj(1)/sj(end);else,x.condition_number=Inf;end
  assert(abs(sum(r.^2)-fval)<=1e-14*max(1,fval));costs(k)=fval;ranges(k)=g.reference_range_m;
 catch ex
  x.error_id=ex.identifier;x.error_message=ex.message;x.error_stack=ex.stack;
 end
 x.iteration_trace=trace(1:traceCount,:);x.objective_evaluations=evals(1:evalCount,:);candidates{k}=x;
 fprintf('%s constrained start %.0fkm -> r0 %.7gkm cost %.7g exit%g, %d/%d infeasible probes, %s\n',model,r0(k)/1000,ranges(k)/1000,costs(k),x.exitflag,sum(x.objective_evaluations(:,5)~=0),evalCount,x.error_id);
end
d=angular_distance_ambiguity(costs,ranges,cfg.near_cost_relative,cfg.near_cost_absolute_rad2,cfg.range_spread_relative);
F=[ones(n,1),c.scaled_time(:)];if c.dimension==9,F=[F,.5*c.scaled_time(:).^2];end;e=c.platform_ENU_m.'-F*(F\c.platform_ENU_m.');
cols=[{'call_index','speed_min_mps','speed_max_mps','acceleration_norm_mps2','violation_code','cost_rad2'},arrayfun(@(j)sprintf('z_%d',j),1:c.dimension,'UniformOutput',false)];
fit=struct('mode',cfg.method,'is_causal',false,'model',model,'context',c,'config',cfg,'candidates',{candidates},'ambiguity',d, ...
 'selection_rule','LOWEST_FINITE_FEASIBLE_ANGULAR_COST_NOT_TRUTH','platform_model_deviation_rms_m',sqrt(sum(e.^2,'all')/n), ...
 'truth_used_in_fit',false,'range_is_validated',false,'prior_usage','HARD_SPEED_ACCEL_NORMS_DISTANCE_INITIALIZATION_ONLY', ...
 'objective_evaluation_columns',{cols});
 function chk=check(z)
  v=z(4:6)*L/T;a=zeros(3,1);if c.dimension==9,a=z(7:9)*L/T^2;end
  chk=motion_kinematic_constraints(v,a,D,cfg.bounds);
 end
 function val=objective(z)
  chk=check(z);evalCount=evalCount+1;
  if chk.feasible,r=angular_motion_residual(z,c);val=sum(r.^2);if ~isfinite(val),error('BO3D:Constrained:NonfiniteObjective','Nonfinite angle objective.');end
  else,val=Inf;end
  evals(evalCount,:)=[evalCount,chk.speed_min_mps,chk.speed_max_mps,chk.acceleration_norm_mps2,chk.violation_code,val,z(:).'];
 end
 function stop=capture(z,values,~)
  stop=false;traceCount=traceCount+1;trace(traceCount,:)=[values.iteration,values.funccount,values.fval,z(:).'];
 end
end
function x=norm3(v),x=hypot(hypot(v(1),v(2)),v(3));end
