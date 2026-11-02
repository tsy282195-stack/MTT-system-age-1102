function result = test_clutter_acceptance_regression()
% Test physical-measurement accounting, delayed confirmation, and missing logs.
K=7; e=repmat(event_template(),K,1); a=cell(K,1);
for k=1:K
    e(k).t_sec=k;e(k).passive.n_meas=1;e(k).passive.ids=0;
    e(k).passive.t_sec=k;e(k).passive.ang=[10;2];e(k).passive.kind=1;
    a{k}=association(1001,1,2,'passive');
end
a{1}.type={'passive_birth'};
a{4}.update_applied=false; % a reserved/rejected update must not count
a{5}=association(-2,2,2,'passive_birth'); % this candidate never confirms
a{6}=struct('id',[],'type',{{}},'meas_index',[]);
a{7}=association(1001,1,2,'passive');
% Duplicate diagnostics of one physical measurement must count once.
names=fieldnames(a{7});for i=1:numel(names),a{7}.(names{i})=[a{7}.(names{i}),a{7}.(names{i})];end
h=struct('basis','actual_confirmation_test','available_dims',[2,3], ...
    'branch_dim',2,'branch_id',1,'event_index',2);
est=struct('assoc',{a},'branch_confirmation_events',h);
c=evaluate_clutter_acceptance(est,e);
assert(c.overall.n_input==7 && c.overall.n_init_success==2);
assert(c.overall.n_update==2 && c.overall.n_accepted==4);
assert(c.overall.n_unconfirmed==1 && c.overall.n_not_used==2);
assert(c.overall.n_duplicate_records==1 && c.three_d.n_input==0);
assert(abs(c.overall.acceptance_rate-4/7)<1e-12);
assert(isnan(c.three_d.acceptance_rate),'An empty population is not zero percent.');
% A passive AE used by a 3-D branch still belongs in the physical 2-D scope.
e3=e(1);e3.active.n_meas=1;e3.active.ids=0;e3.active.has_range=true;
e3.active.rae=[1000;10;2];e3.active.xyz=[1000;0;0];e3.active.t_sec=1;
ap=association(1,10,3,'passive');aa=association(1,10,3,'active');
aa.meas_index=1;b=aa;f=fieldnames(b);
for i=1:numel(f),b.(f{i})=[b.(f{i}),ap.(f{i})];end
est3=struct('assoc',{{b}},'branch_confirmation_events', ...
    struct('basis','actual','available_dims',[2,3],'branch_dim',3,'branch_id',10,'event_index',0));
c3=evaluate_clutter_acceptance(est3,e3);
assert(c3.two_d.n_update==1 && c3.three_d.n_update==1 && c3.overall.n_accepted==2);
% Positive split instance key 1 must not turn raw clutter 0 into real truth.
cfg=config_fusion();cfg.metrics_max_print=0;
labels=build_joint_truth_labels(e,cfg);
assert(all(cellfun(@(x)all(isnan(x)),labels.passive)) && ~any(labels.summary.key_raw_id==0));
cfg.truth_id_split_enabled=false;labels=build_joint_truth_labels(e,cfg);
assert(all(cellfun(@(x)all(isnan(x)),labels.passive)));
% Never infer actual confirmation from a later formal output in old files.
old=rmfield(est,'branch_confirmation_events');old.output_id_map=struct();
missing=evaluate_clutter_acceptance(old,e);
assert(~missing.overall.available && isnan(missing.overall.n_accepted));
% Actual confirmation is available even if no formal output was produced.
angle.logical_tracks={struct('id',1,'confirmed',false),struct('id',1,'confirmed',true)};
space.tracks={[],struct('L',[1;10],'conf',1)};
ch=collect_branch_confirmation_events(space,angle);
assert(isequal(ch.event_index,[2,2]));
integration=actual_filter_boundaries();
result=struct('passed',true,'init_success',c.overall.n_init_success,'updates',c.overall.n_update, ...
    'accepted',c.overall.n_accepted,'input',c.overall.n_input,'actual_filters',integration);
fprintf('杂波接纳回归通过：输入7，成功起始2，确认后更新2，总接纳4；含确认帧、失败起始、拒绝更新和重复记录。\n');
end

function integration=actual_filter_boundaries()
cfg=config_fusion();cfg.metrics_max_print=0;cfg.M_confirm=2;cfg.N_confirm=3;
cfg.joint_confirm_M=2;cfg.joint_confirm_N=3;
cfg.joint_passive_confirm_consecutive_hits=1;cfg.joint_passive_confirm_window_s=0.2;
cfg.assoc_quality_mode='off';cfg.track_quality_mode='off';cfg.bearing_robust_mode='off';
cfg.active_adapt_R_enabled=false;cfg.passive_adapt_R_enabled=false;
cfg.joint_los_trend_enabled=false;cfg.joint_2d_rate_cost_enabled=false;
cfg.joint_2d_stable_lock_enabled=false;cfg.joint_2d_ambiguity_hold_enabled=false;
cfg.use_imm=false;cfg.vel_trend_enabled=false;
platform=struct('lat_deg',30,'lon_deg',120,'alt_m',100, ...
    'interp_lat',@(t)30+zeros(size(t)),'interp_lon',@(t)120+zeros(size(t)), ...
    'interp_alt',@(t)100+zeros(size(t)));
times=(0:7)'*0.1;K=numel(times);out=cell(1,2);
for dim=[2,3]
    xyz=cell(K,1);R=cell(K,1);ids=cell(K,1);pb=cell(K,1);
    meta=repmat(struct('has_active',dim==3,'has_passive',dim==2, ...
        'miss_cycle',dim==3,'confirm_cycle',dim==3,'n_active',double(dim==3), ...
        'n_passive',double(dim==2),'t_start',0,'t_end',0),K,1);
    for k=1:K
        if dim==2
            xyz{k}=zeros(3,0);R{k}=zeros(3,3,0);ids{k}=[];
            pb{k}=struct('ang_deg',[30;4],'R_deg2',diag([0.1,0.1].^2), ...
                'tracklet_id',0,'src',1,'kind',1,'n_meas',1,'t_sec',times(k));
        else
            xyz{k}=[1000+times(k);10000;1000];R{k}=eye(3)*100;ids{k}=0;pb{k}=[];
        end
        meta(k).t_start=times(k);meta(k).t_end=times(k);
    end
    est=struct();events=struct([]);
    captured=evalc('[est,events]=run_filter_joint_legacy_backbone(xyz,R,times,pb,platform,ids,meta,cfg);'); %#ok<NASGU>
    m=evaluate_joint_tracking_metrics(est,events,cfg);c=m.overall.clutter_acceptance;
    assert(c.available && c.n_input==8 && c.n_init_success==2 && c.n_update==6, ...
        'Actual %d-D filter must classify birth+confirmation as init and later updates separately.',dim);
    assert(m.overall.association.n_measurements==8 && m.overall.measurement_accuracy.n_correct==0);
    assert(isempty(m.truth_targets.key_raw_id),'Clutter must not become a truth target.');
    out{dim-1}=c;
end
integration=struct('passive_2d',out{1},'active_3d',out{2});
end

function a=association(public_id,branch_id,dim,type)
a=struct('id',public_id,'type',{{type}},'meas_index',1, ...
    'filter_dim',dim,'usage_branch_dim',dim,'update_applied',~endsWith(type,'birth'), ...
    'branch_2d_id',NaN,'branch_3d_id',NaN);
if dim==2,a.branch_2d_id=branch_id;else,a.branch_3d_id=branch_id;end
end

function e=event_template()
e=struct('t_sec',0,'active',struct('n_meas',0,'ids',[],'has_range',false(1,0), ...
    'rae',zeros(3,0),'xyz',zeros(3,0),'t_sec',[]), ...
    'passive',struct('n_meas',0,'ids',[],'ang',zeros(2,0),'t_sec',[],'kind',[]));
end
