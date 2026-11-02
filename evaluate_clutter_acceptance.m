function result = evaluate_clutter_acceptance(est, events, cfg)
%EVALUATE_CLUTTER_ACCEPTANCE Offline, unique input-measurement clutter counts.
% cfg.clutter_raw_ids defaults to 0; [] disables the clutter convention.
% No truth label changes filter decisions.
% Accepted records are emitted only after a birth/state update is applied.
% The confirmation event itself belongs to initiation, not confirmed update.
if nargin < 3, cfg = struct(); end
clutter_ids = clutter_raw_ids(cfg);
history = confirmation_history(est);
template = struct('event',0,'source','','meas_index',0,'measurement_dim',0, ...
    'track_id',NaN,'branch_dim',0,'branch_id',NaN, ...
    'first_confirmation_event',NaN,'phase',0,'phase_name','not_used', ...
    'duplicate_records',0,'phase_conflict',false);
rows = repmat(template,0,1);
for k = 1:numel(events)
    e = events(k); a = struct();
    if isfield(est,'assoc') && k<=numel(est.assoc) && isstruct(est.assoc{k}), a=est.assoc{k}; end
    for channel = {'active','passive'}
        source=channel{1}; m=e.(source);
        if ~isfield(m,'ids'), continue; end
        for mi = reshape(find(ismember(m.ids,clutter_ids)),1,[])
            if mi>m.n_meas, continue; end
            r=template;r.event=k;r.source=source;r.meas_index=mi;r.measurement_dim=2;
            if strcmp(source,'active') && isfield(m,'has_range') && mi<=numel(m.has_range) && m.has_range(mi)
                r.measurement_dim=3;
            end
            candidates = repmat(template,0,1);
            if isfield(a,'id') && isfield(a,'meas_index') && isfield(a,'type')
                for q=reshape(find(a.meas_index==mi),1,[])
                    type=a.type{q};
                    is_active=strncmp(type,'active',6);
                    is_passive=strncmp(type,'passive',7) || strncmp(type,'companion_passive_',18);
                    if ~(strcmp(source,'active') && is_active || strcmp(source,'passive') && is_passive), continue; end
                    birth=any(strcmp(type,{'active_birth','passive_birth'}));
                    updated=any(strcmp(type,{'active','passive','companion_passive_2d','companion_passive_3d'}));
                    if isfield(a,'update_applied'), updated=logical(at(a.update_applied,q,false)); end
                    if ~birth && ~updated, continue; end
                    c=r;c.track_id=at(a.id,q,NaN);
                    [c.branch_dim,c.branch_id]=receiver(a,q,r.measurement_dim);
                    if ~ismember(c.branch_dim,history.available_dims) || ~isfinite(c.branch_id)
                        c.phase=4;c.phase_name='confirmation_history_unavailable';
                    else
                        j=find(history.branch_dim==c.branch_dim & history.branch_id==c.branch_id,1);
                        if isempty(j)
                            c.phase=3;c.phase_name='unconfirmed_in_observed_window';
                        else
                            c.first_confirmation_event=history.event_index(j);
                            if k<=c.first_confirmation_event
                                c.phase=1;c.phase_name='successful_initiation';
                            elseif updated && ~birth
                                c.phase=2;c.phase_name='confirmed_update';
                            else
                                c.phase=4;c.phase_name='inconsistent_birth_history';
                            end
                        end
                    end
                    candidates(end+1)=c; %#ok<AGROW>
                end
            end
            if ~isempty(candidates)
                % One physical measurement is counted once, even if duplicated
                % in diagnostics. A confirmed update takes precedence if it
                % also seeded a different branch in the same event.
                phases=[candidates.phase];
                j=find(phases==2,1);if isempty(j),j=find(phases==1,1);end
                if isempty(j),j=find(phases==4,1);end
                if isempty(j),j=1;end
                r=candidates(j);r.duplicate_records=numel(candidates)-1;
                r.phase_conflict=any(phases==1) && any(phases==2);
            end
            rows(end+1)=r; %#ok<AGROW>
        end
    end
end
result=struct('schema_version',2,'clutter_raw_id',clutter_ids, ...
    'clutter_raw_ids',clutter_ids,'definition_enabled',~isempty(clutter_ids), ...
    'phase_rule','event<=first_confirmation:init;later_actual_update:update', ...
    'confirmation_history_basis',history.basis,'records',rows);
result.two_d=summarize(rows,[rows.measurement_dim]==2,clutter_ids);
result.three_d=summarize(rows,[rows.measurement_dim]==3,clutter_ids);
result.overall=summarize(rows,true(1,numel(rows)),clutter_ids);
result.default_track=summarize(rows,false(1,numel(rows)),clutter_ids);
result.default_track.n_input=result.overall.n_input;
result.default_track=ratios(result.default_track);
ids=unique([rows.track_id]);ids=ids(isfinite(ids));
result.by_track=repmat(struct('track_id',0,'metrics',struct()),0,1);
for id=reshape(ids,1,[])
    owned=[rows.track_id]==id;
    c=summarize(rows,owned,clutter_ids);c.n_input=result.overall.n_input;
    c=ratios(c);
    result.by_track(end+1)=struct('track_id',id,'metrics',c); %#ok<AGROW>
end
assert(result.overall.n_input==result.two_d.n_input+result.three_d.n_input);
end

function h=confirmation_history(est)
if isfield(est,'branch_confirmation_events')
    h=est.branch_confirmation_events;return;
end
% Full native histories can be evaluated exactly. Compacted public outputs
% cannot stand in for the missing internal confirmation history.
if ~isfield(est,'output_id_map') && isfield(est,'logical_tracks')
    h=collect_branch_confirmation_events(struct(),est,true);
elseif ~isfield(est,'output_id_map') && isfield(est,'tracks')
    h=collect_branch_confirmation_events(est,struct());
else
    h=collect_branch_confirmation_events(struct(),struct());
end
end

function [dim,id]=receiver(a,q,physical_dim)
dim=at_field(a,'usage_branch_dim',q,at_field(a,'filter_dim',q,physical_dim));
id=NaN;
if dim==3,id=at_field(a,'branch_3d_id',q,NaN);
elseif dim==2,id=at_field(a,'branch_2d_id',q,NaN);end
if ~ismember(dim,[2,3])
    id=at_field(a,'branch_2d_id',q,NaN);dim=2;
    if ~isfinite(id),id=at_field(a,'branch_3d_id',q,NaN);dim=3;end
end
if ~isfinite(id),id=at_field(a,'id',q,NaN);end
end

function c=summarize(rows,mask,clutter_ids)
p=[rows(mask).phase];
c=struct('basis','unique_configured_clutter_input_measurements', ...
    'clutter_raw_ids',clutter_ids,'definition_enabled',~isempty(clutter_ids), ...
    'status','ok','available',true,'n_input',nnz(mask), ...
    'n_init_success',nnz(p==1),'n_update',nnz(p==2), ...
    'n_accepted',nnz(p==1 | p==2),'n_unconfirmed',nnz(p==3), ...
    'n_not_used',nnz(p==0),'n_unresolved',nnz(p==4), ...
    'n_duplicate_records',sum([rows(mask).duplicate_records]), ...
    'n_phase_conflicts',nnz([rows(mask).phase_conflict]));
if c.n_unresolved>0
    c.status='unavailable_confirmation_history';c.available=false;
    c.n_init_success=NaN;c.n_update=NaN;c.n_accepted=NaN;
elseif isempty(clutter_ids)
    c.status='clutter_definition_disabled';
elseif c.n_input==0
    c.status='no_clutter_samples';
end
c=ratios(c);
if c.available
    assert(c.n_init_success+c.n_update==c.n_accepted && c.n_accepted<=c.n_input);
end
end

function c=ratios(c)
c.init_acceptance_rate=NaN;c.update_acceptance_rate=NaN;c.acceptance_rate=NaN;
if c.n_input>0 && c.available
    c.init_acceptance_rate=c.n_init_success/c.n_input;
    c.update_acceptance_rate=c.n_update/c.n_input;
    c.acceptance_rate=c.n_accepted/c.n_input;
end
end

function value=at_field(s,name,q,fallback)
value=fallback;if isfield(s,name),value=at(s.(name),q,fallback);end
end
function value=at(x,q,fallback)
value=fallback;if q>=1 && q<=numel(x),value=x(q);end
end
