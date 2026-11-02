function result=test_range_assist_regression(old_dir)
% Focused behavioral contracts; no assertion of true-distance accuracy.
if nargin<1,old_dir='';end
checks={};cfg=fixture_config();o=range_assist_options(cfg);
bad=cfg;bad.joint_range_assist_weight=1.01;expect_error(@()range_assist_options(bad),'RangeAssist:InvalidConfig');
bad=cfg;bad.joint_range_assist_max_samples=10;expect_error(@()range_assist_options(bad),'RangeAssist:InvalidConfig');
checks{end+1}='configuration_validation';
range_assist_toolkit();

% Independent geometric system with two different candidate continuations.
s=manual_cache(0,30,4);o.enabled=true;
[c0,d0]=range_assist_pair_cost(s,.1,zeros(3,1),[30;4],[30;4],o);
[c1,d1]=range_assist_pair_cost(s,.1,zeros(3,1),[30;4],[30.01;4.002],o);
[c2,d2]=range_assist_pair_cost(s,.1,zeros(3,1),[30;4],[30.02;4.005],o);
assert(d0.available&&d1.available&&d2.available&&c0==0&&c1>0&&abs(c1-c2)>eps);
assert(abs(d1.reference_range_m-norm(s.origin_ENU_m+1e5*(s.z(1:3)+s.z(4:6)*.1/20)))<1e-8);
[~,x]=range_assist_pair_cost(s,0,zeros(3,1),[30;4],[30;4],o);assert(strcmp(x.reason,'CACHE_NOT_PRIOR'));
[~,x]=range_assist_pair_cost(s,7,zeros(3,1),[30;4],[30;4],o);assert(strcmp(x.reason,'CACHE_EXPIRED'));
[cw,dw]=range_assist_pair_cost(s,.1,zeros(3,1),[179.99;4],[-179.99;4],o);
[ce,de]=range_assist_pair_cost(s,.1,zeros(3,1),[179.99;4],[180.01;4],o);
assert(dw.available&&de.available&&abs(cw-ce)<1e-12);
checks{end+1}='candidate_geometry_wrap_and_cache_age';

% Posteriors have a 12-s history; two budget-stopped fits remain candidates.
o.initial_ranges_m=70000;o.max_iterations=1;o.max_function_evaluations=80;
u=[];reports=struct([]);
for t=0:.05:12
 p=[4000*sin(t/4);3000*(cos(t/5)-1);11000+800*sin(t/3)];
 q=[20000;30000;12000]+[250;40;5]*t-p;
 ae=[atan2d(q(1),q(2));atan2d(q(3),hypot(q(1),q(2)))];
 [u,r]=range_assist_update(u,1,1,t,ae,p,o);
 if ~isempty(r),reports=[reports,r];end %#ok<AGROW>
end
assert(u.available&&u.attempts==2&&u.successes==2&&numel(reports)==2);
assert(abs(reports(1).time_s-10)<1e-8&&abs(reports(2).time_s-12)<1e-8);
assert(~u.range_is_validated&&u.solver_exitflag~=1&&u.history(1,end)==12);
old_cutoff=u.data_cutoff_s;o.initial_ranges_m=realmax;
for t=12.05:.05:14
 [u,r]=range_assist_update(u,1,1,t,[30;4],zeros(3,1),o);
end
assert(u.failures==1&&u.data_cutoff_s==old_cutoff&&u.available&&~isempty(r.error_id));
[gap,~]=range_assist_update(u,1,1,14.2,[30;4],zeros(3,1),o);
assert(~gap.available&&size(gap.history,2)==1);
[born,~]=range_assist_update(u,1,2,14.05,[30;4],zeros(3,1),o);
assert(born.attempts==0&&size(born.history,2)==1);
checks{end+1}='refresh_schedule_budget_failure_gap_and_identity';
ca=range_assist_empty();co=o;co.model='CA';co.initial_ranges_m=70000;
for t=0:.05:10,[ca,car]=range_assist_update(ca,3,3,t,[30+.01*t;4],zeros(3,1),co);end
assert(ca.available&&numel(ca.z)==9&&strcmp(ca.model,'CA')&&isempty(car.error_id));
[cc,ca_detail]=range_assist_pair_cost(ca,10.05,zeros(3,1),[30.1;4],[30.1001;4],co);
assert(ca_detail.available&&isfinite(cc));checks{end+1}='CA_refresh_and_continuation';

% Full kernel: off/zero, stable lock bypass, residual ranking, causality.
events=arrayfun(@(t)event(t,[30+.01*t;4]),(0:.05:11)','UniformOutput',false);events=vertcat(events{:});
platform=fixture_platform();
[off,off_state]=run_filter_joint_2d3d(events,platform,cfg);
cfg.joint_range_assist_enabled=true;cfg.joint_range_assist_weight=0;
cfg.joint_range_assist_initial_ranges_m=70000;
cfg.joint_range_assist_max_iterations=1;cfg.joint_range_assist_max_function_evaluations=80;
[zero,zero_state]=run_filter_joint_2d3d(events,platform,cfg);
assert(isequaln(off.assoc,zero.assoc)&&isequaln(off.output,zero.output));
assert(isequaln(off_state.tracks(1).angle,zero_state.tracks(1).angle));
assert(zero_state.stats.range_assist.successes==1&&off_state.stats.range_assist.attempts==0);
checks{end+1}='zero_weight_exact_filter_and_assignment_parity';
cfg.joint_range_assist_weight=.1;
[batch,batch_state]=run_filter_joint_2d3d(events,platform,cfg);
[first,state]=run_filter_joint_2d3d(events(1:130),platform,cfg);
[second,state]=run_filter_joint_2d3d(events(131:end),platform,cfg,state);
assert(isequaln(batch.assoc,[first.assoc;second.assoc]));
assert(isequaln(batch_state.tracks,state.tracks));
future=events;future(end).passive.ang=[31;4.2];
[future_est,~]=run_filter_joint_2d3d(future,platform,cfg);
assert(isequaln(batch.assoc(1:end-1),future_est.assoc(1:end-1)));
assert(batch_state.stats.range_assist.edges_applied>0);
checks{end+1}='stream_resume_and_prefix_causality';

% Force a populated two-track prior; distance cannot touch a locked pair.
c=fixture_config();c.joint_range_assist_enabled=true;c.joint_range_assist_weight=1;
c.joint_range_assist_diagnostic_level='edges';c.joint_range_assist_diagnostic_event_indices=2;
[~,seed]=run_filter_joint_2d3d(event(0,[30,30.3;4,4.1]),platform,c);
for i=1:2
 seed.tracks(i).range_assist=manual_cache(0,30+.3*(i-1),4+.1*(i-1));
 seed.tracks(i).range_assist.identity=[seed.tracks(i).id,seed.tracks(i).birth_event];
end
e=event(.05,[30,30.3;4,4.1]);c.joint_2d_stable_lock_enabled=true;c.joint_2d_lock_margin=0;
seed.params.stable_lock_enabled=true;seed.params.lock_margin=0;
[locked,~]=run_filter_joint_2d3d(e,platform,c,seed);
r=locked.range_assist{1}.associations;
assert(numel(r.track_id)==2&&all(r.locked)&&~any(r.distance_applied));
assert(isequal([r.original_cost],[r.final_cost]));
mapped=map_range_diagnostics(locked.range_assist, ...
 {struct('branch_2d_id',r.branch_2d_id,'meas_index',[5,9],'id',[7001,7002])},{[5,9]});
assert(isequal(mapped{1}.associations.meas_index,[5,9])&& ...
 isequal(mapped{1}.associations.track_id,[7001,7002]));
legend=range_assist_reason();assert(strcmp(legend{double(r.reason_code(1))+1},'STABLE_LOCK_BYPASS'));
checks{end+1}='columnar_diagnostic_public_id_and_measurement_mapping';
c.joint_2d_stable_lock_enabled=false;
seed.params.stable_lock_enabled=false;
[unlocked,~]=run_filter_joint_2d3d(e,platform,c,seed);
d=unlocked.range_assist{1}.edges;
assert(d.n_applied>0&&any(d.original_cost(d.applied)~=d.final_cost(d.applied)));
% A controlled ranking example verifies the normalized contribution can
% change a global assignment while preserving the same feasible graph.
A=[1,2;2,1];D=[1,0;0,1];
assert(isequal(solve_global_assignment(A,250),[1,1;2,2]));
assert(isequal(solve_global_assignment(.9*A+.1*250*D,250),[1,2;2,1]));
checks{end+1}='stable_lock_bypass_and_unlocked_cost_participation';
stale=seed;for i=1:2,stale.tracks(i).range_assist.data_cutoff_s=-10;end
[fallback,~]=run_filter_joint_2d3d(e,platform,c,stale);
r=fallback.range_assist{1}.associations;assert(~any([r.distance_applied]));
assert(isequal([r.original_cost],[r.final_cost]));
checks{end+1}='unavailable_feature_original_cost_fallback';
[no_platform,~]=run_filter_joint_2d3d(e,[],c,seed);
r=no_platform.range_assist{1}.associations;
assert(~any(r.distance_applied)&&isequal(r.original_cost,r.final_cost));
checks{end+1}='missing_platform_does_not_use_cached_distance';

% Old streams acquire empty range state without changing their old evidence.
legacy=rmfield(seed,'schema_version');legacy.tracks=rmfield(legacy.tracks,'range_assist');
legacy.params=rmfield(legacy.params,'range_assist');legacy.stats=rmfield(legacy.stats,'range_assist');
[~,resumed]=run_filter_joint_2d3d(e,platform,c,legacy);assert(resumed.schema_version==16);
checks{end+1}='legacy_stream_upgrade';
% Review drops only repeated saved confirmation windows; live filtering and
% full history retain them. The two production executions must agree.
hc=fixture_config();hc.joint_extension_enabled=true;hc.joint_range_assist_enabled=true;
hc.joint_range_assist_weight=0;tt=(0:7)'*.05;K=numel(tt);
xyz=repmat({zeros(3,0)},K,1);rr=repmat({zeros(3,3,0)},K,1);ids=repmat({[]},K,1);pb=cell(K,1);
meta=repmat(struct('has_active',false,'has_passive',true,'miss_cycle',false, ...
 'confirm_cycle',false,'n_active',0,'n_passive',1,'t_start',0,'t_end',0),K,1);
for k=1:K
 pb{k}=struct('ang_deg',[30;4],'R_deg2',diag([.1,.05].^2),'tracklet_id',101, ...
  'src',1,'kind',1,'n_meas',1,'t_sec',tt(k));
 meta(k).t_start=tt(k);meta(k).t_end=tt(k);
end
hc.result_save_mode='review';text=evalc('rv=run_filter_adapt_ckf(xyz,rr,tt,hc,pb,platform,ids,meta);'); %#ok<NASGU>
hc.result_save_mode='full';text=evalc('fl=run_filter_adapt_ckf(xyz,rr,tt,hc,pb,platform,ids,meta);'); %#ok<NASGU>
assert(isequaln(rv.joint2d.assoc,fl.joint2d.assoc));
for k=1:K
 projected=rmfield(fl.joint2d.output{k},setdiff(fieldnames(fl.joint2d.output{k}),fieldnames(rv.joint2d.output{k})));
 assert(isequaln(rv.joint2d.output{k},projected));
end
assert(~isempty(rv.joint2d.logical_tracks{end})&&~isfield(rv.joint2d.logical_tracks{end},'hit_history'));
assert(~isempty(fl.joint2d.logical_tracks{end}(1).hit_history));
assert(isequaln(rv.joint2d.final_noise_snapshots(1).measurement_noise,fl.joint2d.logical_tracks{end}(1).measurement_noise));
checks{end+1}='review_history_compaction_preserves_production_filter';
hc.result_save_mode='review';text=evalc('[rp,~]=run_filter_joint_legacy_backbone(xyz,rr,tt,pb,platform,ids,meta,hc);'); %#ok<NASGU>
hc.result_save_mode='full';text=evalc('[fp,~]=run_filter_joint_legacy_backbone(xyz,rr,tt,pb,platform,ids,meta,hc);'); %#ok<NASGU>
assert(isequaln(rp.assoc,fp.assoc)&&isequaln(rp.measurement_noise,fp.measurement_noise));
assert(isequaln(rp.branch_confirmation_events,fp.branch_confirmation_events));
assert(isequaln(joint_review_output('pack',rp.output),joint_review_output('pack',fp.output)));
checks{end+1}='public_review_output_noise_and_confirmation_parity';
if ~isempty(old_dir)
 c=fixture_config();before=path;old_cwd=pwd;addpath(old_dir,'-begin');cd(old_dir);clear run_filter_joint_2d3d
 [old,old_state]=run_filter_joint_2d3d(events(1:30),platform,c);
 cd(old_cwd);path(before);clear run_filter_joint_2d3d
 [now,new_state]=run_filter_joint_2d3d(events(1:30),platform,c);
 assert(isequaln(old.assoc,now.assoc)&&isequaln(old.output,now.output));
 assert(isequaln(old_state.tracks,rmfield(new_state.tracks,'range_assist')));
 checks{end+1}='original_kernel_exact_disabled_parity';
end
result=struct('passed',true,'checks',{checks},'n_checks',numel(checks),'range_is_validated',false);
fprintf('Range-assist regression: %d behavioral contracts PASS\n',numel(checks));
end
function s=manual_cache(t,az,el)
s=range_assist_empty(1,1);s.available=true;s.data_cutoff_s=t;s.reference_time_s=t;
s.origin_ENU_m=zeros(3,1);s.length_m=1e5;s.time_scale_s=20;s.model='CV';
u=[cosd(el)*sind(az);cosd(el)*cosd(az);sind(el)];
s.z=[.7*u;[250;40;5]*20/1e5];s.history_R=eye(6)*.1;
end
function cfg=fixture_config()
cfg=config_fusion();cfg.joint_range_assist_mode='batch';
cfg.joint_range_assist_enabled=false;
cfg.joint_range_assist_weight=.1;
cfg.joint_range_assist_window_s=20;
cfg.joint_range_assist_min_span_s=10;cfg.joint_range_assist_interval_s=2;
cfg.joint_range_assist_sample_interval_s=.05;cfg.joint_range_assist_max_samples=501;
cfg.joint_range_assist_scale_m=20000;
cfg.joint_confirm_M=1;cfg.joint_confirm_N=1;
cfg.joint_passive_confirm_group_size=1;cfg.joint_streaming_quiet=true;
cfg.joint_los_trend_enabled=false;cfg.joint_2d_rate_cost_enabled=false;
cfg.joint_2d_stable_lock_enabled=false;cfg.joint_2d_ambiguity_hold_enabled=false;
cfg.assoc_quality_mode='off';cfg.track_quality_mode='off';cfg.bearing_robust_mode='off';
cfg.joint_angle95_max_deg=10;cfg.joint_gate_2d=100;cfg.joint_2d_accept_nis=100;
end
function p=fixture_platform()
p=struct('lat_deg',30,'lon_deg',120,'alt_m',100,'interp_lat',@(t)30+zeros(size(t)), ...
 'interp_lon',@(t)120+zeros(size(t)),'interp_alt',@(t)100+zeros(size(t)));
end
function e=event(t,ang)
a=struct('t_sec',zeros(1,0),'xyz',zeros(3,0),'rae',zeros(3,0), ...
 'R_xyz',zeros(3,3,0),'R_ae',zeros(2,2,0),'has_range',false(1,0),'ids',zeros(1,0),'src',zeros(1,0),'n_meas',0);
n=size(ang,2);p=struct('t_sec',t,'ang',ang,'R_ae',repmat(diag([.1,.05].^2),1,1,n), ...
 'ids',101:100+n,'src',ones(1,n),'kind',ones(1,n),'original_index',1:n,'n_meas',n);
e=struct('cycle_id',1,'t_sec',t,'t_start',t,'t_end',t,'confirm_cycle',true, ...
 'has_active',false,'has_passive',true,'active',a,'passive',p);
end
function expect_error(f,id)
try,f();catch ex,assert(strcmp(ex.identifier,id));return;end
error('RangeAssist:ExpectedError','Expected %s.',id);
end
