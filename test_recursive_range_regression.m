function result=test_recursive_range_regression()
% Behavioral checks for causal short-window ranging and transactional trials.
cfg=config_fusion();cfg.joint_range_assist_enabled=true;
cfg.joint_range_assist_window_s=20; % Keep the complete synthetic history.
cfg.joint_range_assist_initial_ranges_m=[70000,100000];
cfg.joint_range_assist_max_iterations=80;cfg.joint_range_assist_max_function_evaluations=240;
o=range_assist_options(cfg);R=diag([.1,.05].^2);s=[];first=NaN;
for t=0:.01:2.2
 sensor=[4000*sin(t/4);3000*(cos(t/5)-1);11000+800*sin(t/3)];
 relative=[20000;30000;12000]+[250;40;5]*t-sensor;
 z=angles(relative);
 [s,report]=range_assist_update(s,1,1,t,z,sensor,o,R);
 if s.available&&~isfinite(first),first=t;end
end
assert(s.available&&abs(first-2)<1e-8&&s.n_recursive_updates>=19);
assert(isequal(s.history(2:3,end),z)&&size(s.history,2)==221);
assert(s.range_std_m>0&&s.quality>0&&s.quality<=1&&~s.range_is_validated);
% All candidate evaluations use the same prior and cannot alter it.
before=s;t=2.21;prior=range_assist_recursive('predict',s,t,sensor,o);
z0=angles(prior.models(1).x(1:3)-sensor);
[c1,d1]=range_assist_recursive('candidate',prior,z0+[.01;.002],R,o);
[c2,d2]=range_assist_recursive('candidate',prior,z0+[.03;.004],R,o);
[again,da]=range_assist_recursive('candidate',prior,z0+[.01;.002],R,o);
assert(isequaln(s,before)&&isequaln(c1,again)&&isequaln(d1,da));
assert(d1.available&&d2.available&&isfinite(c1)&&isfinite(c2)&&abs(c1-c2)>1e-15);
fine=o;fine.recursive_iterations=10;
[~,df]=range_assist_recursive('candidate',prior,z0+[.01;.002],R,fine);
assert(abs(df.candidate_range_m-d1.candidate_range_m)<10, ...
 'Bounded iterations diverged from the converged local prior+measurement fit.');
[~,wrap1]=range_assist_recursive('candidate',prior,[-179.99;4],R,o);
[~,wrap2]=range_assist_recursive('candidate',prior,[180.01;4],R,o);
assert(isequaln(wrap1,wrap2));
% A candidate cannot access a current/future cache.
future=range_assist_recursive('predict',s,s.data_cutoff_s,sensor,o);
assert(~future.available&&strcmp(future.reason,'CACHE_NOT_PRIOR'));
% A jump is scored against the old model, then quarantined AFTER selection.
[frozen,rr]=range_assist_update(s,1,1,2.21,z+[1;.21],sensor,o,R);
assert(isequaln(frozen.models,s.models)&&frozen.data_cutoff_s==s.data_cutoff_s);
assert(strcmp(rr.kind,'frozen')&&size(frozen.pending_history,2)==1);
for t=2.26:.05:4.21
 [frozen,rr]=range_assist_update(frozen,1,1,t,z+[1;.21],sensor,o,R);
end
assert(frozen.available&&frozen.initialized_t>=4.2&&all(diff(frozen.history(1,:))>0), ...
 'Quarantined segments must recover without duplicate timestamps.');
[newborn,~]=range_assist_update(s,1,2,2.21,z,sensor,o,R);
assert(~newborn.available&&size(newborn.history,2)==1);
[gap,~]=range_assist_update(s,1,1,3,z,sensor,o,R);
assert(~gap.available&&size(gap.history,2)==1);
quick=o;quick.min_span_s=1;short=[];
for t=0:.05:1
 [short,~]=range_assist_update(short,2,1,t,z,sensor,quick,R);
end
assert(short.available&&abs(short.initialized_t-1)<1e-8);
warm=[];
for t=0:.05:2
 a=z;if t>=1,a=a+[1;.21];end
 [warm,~]=range_assist_update(warm,3,1,t,a,sensor,o,R);
end
assert(~warm.available&&abs(warm.history(1,1)-1)<1e-8, ...
 'Initialization must not fit across an angle step.');
% Whole-component fallback; a disconnected, fully ready component still works.
A=[1,2,inf;2,1,inf;inf,inf,1];C=ones(3);q=ones(3);ready=isfinite(A);ready(1,2)=false;
reason=repmat({'RECURSIVE_CONTINUATION'},3);
[B,applied]=range_assist_consistent_cost(A,C,q,ready,reason,.2);
assert(isequal(B(1:2,1:2),A(1:2,1:2))&&~any(applied(1:2,1:2),'all'));
assert(B(3,3)==1.2&&applied(3,3));
A=[1,2;2,1];D=[9,0;0,9];ready=true(2);reason=repmat({'RECURSIVE_CONTINUATION'},2);
[B,applied]=range_assist_consistent_cost(A,D,ones(2),ready,reason,5,250,.01);
assert(isequal(B,A)&&~any(applied,'all'),'Range must not override a decisive angular assignment.');
A=[1,1.0001;1.0001,1];
[B,applied]=range_assist_consistent_cost(A,D,ones(2),ready,reason,5,250,.01);
assert(all(applied,'all')&&isequal(solve_global_assignment(B,250),[1,2;2,1]));

% Exercise the actual filter: raw observations, zero-weight parity, resume,
% immutable output prefixes, locked-edge bypass and public ID diagnostics.
cfg.joint_confirm_M=1;cfg.joint_confirm_N=1;cfg.joint_passive_confirm_group_size=1;
cfg.joint_streaming_quiet=true;cfg.joint_los_trend_enabled=false;
cfg.joint_2d_rate_cost_enabled=false;cfg.joint_2d_stable_lock_enabled=false;
cfg.joint_2d_ambiguity_hold_enabled=false;cfg.assoc_quality_mode='off';
cfg.track_quality_mode='off';cfg.bearing_robust_mode='off';
cfg.joint_range_assist_weight=0;
events=arrayfun(@(t)event(t,[30+.05*t;4]),(0:.05:2.5)','UniformOutput',false);
events=vertcat(events{:});platform=fixture_platform();
[zero,state]=run_filter_joint_2d3d(events,platform,cfg);
assert(state.tracks(1).range_assist.available);
assert(isequal(state.tracks(1).range_assist.history(2:3,end),events(end).passive.ang));
off_cfg=cfg;off_cfg.joint_range_assist_enabled=false;
[off,~]=run_filter_joint_2d3d(events,platform,off_cfg);
assert(isequaln(zero.assoc,off.assoc)&&isequaln(zero.output,off.output));
cfg.joint_range_assist_weight=.1;
[full,full_state]=run_filter_joint_2d3d(events,platform,cfg);
[a,part]=run_filter_joint_2d3d(events(1:30),platform,cfg);
[b,part]=run_filter_joint_2d3d(events(31:end),platform,cfg,part);
assert(isequaln(full.assoc,[a.assoc;b.assoc])&&isequaln(full_state.tracks,part.tracks));
changed=events;changed(end).passive.ang=[31;4.2];
[future_est,~]=run_filter_joint_2d3d(changed,platform,cfg);
assert(isequaln(full.assoc(1:end-1),future_est.assoc(1:end-1)));
assert(full_state.stats.range_assist.recursive_updates>0);
assert(~isempty(full.range_assist{end}.estimates));
locked_cfg=cfg;locked_cfg.joint_2d_stable_lock_enabled=true;
locked=full_state;locked.params.stable_lock_enabled=true;locked.params.lock_margin=0;
[one,~]=run_filter_joint_2d3d(event(2.55,[30+.05*2.55;4]),platform,locked_cfg,locked);
assert(all(one.range_assist{1}.associations.locked)&&~any(one.range_assist{1}.associations.distance_applied));
mapped=map_range_diagnostics(one.range_assist, ...
 {struct('branch_2d_id',one.range_assist{1}.estimates.branch_2d_id,'meas_index',5,'id',7001)}, {5});
assert(mapped{1}.estimates.track_id==7001);
folder=fullfile(tempdir,['recursive_range_test_',char(java.util.UUID.randomUUID)]);
files=export_range_diagnostics(full,1,folder);
assert(files.available&&isfile(files.state_files{1})&&isfile(files.association_files{1}));
% Remove only the two exact temporary files created by this test.
delete(files.state_files{1});delete(files.association_files{1});rmdir(folder);
result=struct('passed',true,'first_initialization_s',first, ...
 'n_recursive_updates',s.n_recursive_updates,'range_std_m',s.range_std_m, ...
 'candidate_costs',[c1,c2],'bounded_iteration_range_error_m',abs(df.candidate_range_m-d1.candidate_range_m), ...
 'zero_weight_parity',true,'stream_and_prefix_parity',true,'component_fallback',true);
disp(result);
end
function a=angles(q)
a=[atan2d(q(1),q(2));atan2d(q(3),hypot(q(1),q(2)))];
end
function p=fixture_platform()
p=struct('lat_deg',30,'lon_deg',120,'alt_m',100,'interp_lat',@(t)30+zeros(size(t)), ...
 'interp_lon',@(t)120+zeros(size(t)),'interp_alt',@(t)100+zeros(size(t)));
end
function e=event(t,z)
e=struct('cycle_id',1,'t_sec',t,'t_start',t,'t_end',t,'confirm_cycle',true, ...
 'has_active',false,'has_passive',true,'active',struct('t_sec',[], ...
 'xyz',zeros(3,0),'rae',zeros(3,0),'R_xyz',zeros(3,3,0),'R_ae',zeros(2,2,0), ...
 'has_range',false(1,0),'ids',[],'src',[],'n_meas',0), ...
 'passive',struct('t_sec',t,'ang',z,'R_ae',diag([.1,.05].^2),'ids',101, ...
 'src',1,'kind',1,'original_index',1,'n_meas',1));
end
