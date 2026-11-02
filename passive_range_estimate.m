function [result,diagnostics]=passive_range_estimate(input,options)
%PASSIVE_RANGE_ESTIMATE Offline constrained CV/CA angle-domain fit, no truth.
expected={'model';'apply_smoothing';'fit';'smoothing'};
if ~isstruct(options)||~isscalar(options)||~isequal(sort(fieldnames(options)),sort(expected))|| ...
 ~islogical(options.apply_smoothing)||~isscalar(options.apply_smoothing)
 error('PassiveToolkit:InvalidOptions','Exact explicit options struct required.');
end
angular_motion_context(input,options.model,options.fit.scales);
used=input;sd=[];
if options.apply_smoothing,[used,sd]=smooth_passive_angles(input,options.smoothing);end
fit=fit_constrained_angular_motion(used,options.model,options.fit);
t=used.time_s(:).';n=numel(t);blank=NaN(3,n);
result=struct('schema_version','portable-passive-range-result-v1','method','OFFLINE_ANGLE_BATCH', ...
 'is_causal',false,'coordinate_frame','local_enu','model',options.model,'time_s',t, ...
 'range_candidate_m',NaN(1,n),'target_position_ENU_m',blank,'relative_position_ENU_m',blank, ...
 'target_velocity_ENU_mps',blank,'target_acceleration_ENU_mps2',blank, ...
 'candidate_available',false,'solver_converged',false,'solver_exitflag',NaN, ...
 'selected_candidate_index',[],'angular_cost_rad2',NaN,'ambiguity_detected',fit.ambiguity.ambiguity_detected, ...
 'status','NO_FINITE_CANDIDATE','preprocessing','RAW','range_is_validated',false);
if options.apply_smoothing,result.preprocessing='LOCAL_QUADRATIC_OFFLINE';end
k=fit.ambiguity.best_index;
if ~isempty(k)
 x=fit.candidates{k};g=x.geometry;
 result.range_candidate_m=g.range_m;result.target_position_ENU_m=g.position_ENU_m;
 result.relative_position_ENU_m=g.relative_position_ENU_m;result.target_velocity_ENU_mps=g.velocity_ENU_mps;
 result.target_acceleration_ENU_mps2=repmat(g.acceleration_ENU_mps2,1,n);
 result.candidate_available=true;result.solver_exitflag=x.exitflag;result.solver_converged=x.exitflag==1;
 result.selected_candidate_index=k;result.angular_cost_rad2=x.cost_rad2;
 if result.solver_converged,result.status='CANDIDATE_ONLY_NOT_DISTANCE_VALIDATED';
 else,result.status='CANDIDATE_SOLVER_NOT_CONVERGED';end
end
diagnostics=struct('fit',fit,'smoothing',sd,'input_used',used,'options',options, ...
 'truth_used',false,'range_is_validated',false);
end
