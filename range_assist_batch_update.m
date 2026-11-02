function [s,report]=range_assist_batch_update(s,id,birth_event,t,ae_deg,sensor,o)
% Confirmed, actually updated angle posteriors only. Called AFTER assignment.
% The returned fit becomes available to the NEXT event, never this event.
report=[];
if ~o.enabled,return;end
if isempty(s)||~isequal(s.identity,[id,birth_event]),s=range_assist_empty(id,birth_event);end
if any(~isfinite([t;ae_deg(:);sensor(:)]))||numel(ae_deg)~=2||numel(sensor)~=3
 s.status='INVALID_SAMPLE';return;end
if isfinite(s.last_sample_t)&&t<=s.last_sample_t+o.sample_interval_s-1e-9,return;end
if ~isempty(s.history)&&t-s.history(1,end)>.08+1e-9
 % Preserve the toolkit gap policy. A new contiguous segment needs warmup.
 s.history=zeros(6,0);s.available=false;s.status='WARMUP_AFTER_GAP';
end
s.history(:,end+1)=[t;ae_deg(:);sensor(:)];s.last_sample_t=t;
s.history=s.history(:,s.history(1,:)>=t-o.window_s-1e-9);
if size(s.history,2)>o.max_samples,s.history=s.history(:,end-o.max_samples+1:end);end
if size(s.history,2)<8||s.history(1,end)-s.history(1,1)<o.min_span_s-1e-9,return;end
if isfinite(s.last_attempt_t)&&t-s.last_attempt_t<o.interval_s-1e-9,return;end
s.last_attempt_t=t;s.attempts=s.attempts+1;started=tic;
report=struct('track_id',id,'birth_event',birth_event,'time_s',t, ...
 'window_start_s',s.history(1,1),'data_cutoff_s',s.history(1,end), ...
 'n_samples',size(s.history,2),'status','FAILED','error_id','','error_message','', ...
 'selected_candidate_index',[],'candidates',[],'elapsed_s',NaN, ...
 'range_is_validated',false,'full_fit',[]);
try
 range_assist_toolkit(true,true);
 input=make_passive_range_input(s.history(1,:),s.history(2,:),s.history(3,:),s.history(4:6,:),'deg');
 options=passive_range_options();options.model=o.model;options.apply_smoothing=true;
 options.smoothing.half_window_s=o.smoothing_half_window_s;
 options.fit.min_span_s=o.min_span_s;
 options.fit.max_iterations=o.max_iterations;options.fit.max_function_evaluations=o.max_function_evaluations;
 options.fit.initial_ranges_m=o.initial_ranges_m;options.fit.bounds=o.bounds;
 % The kernel emits one line per start. Keep it out of the per-event console.
 transcript=evalc('[result,d]=passive_range_estimate(input,options);'); %#ok<NASGU>
 report.selected_candidate_index=result.selected_candidate_index;
 report.candidates=cellfun(@candidate_summary,d.fit.candidates,'UniformOutput',false);
 if strcmp(o.diagnostic_level,'full'),report.full_fit=d;end
 if ~result.candidate_available,error('RangeAssist:NoCandidate','No finite feasible motion candidate.');end
 c=d.fit.candidates{result.selected_candidate_index};ctx=d.fit.context;
 [~,R]=qr(c.jacobian,0);
 s.available=true;s.data_cutoff_s=s.history(1,end);s.window_start_s=s.history(1,1);
 s.reference_time_s=ctx.reference_time_s;s.origin_ENU_m=ctx.origin_ENU_m;
 s.z=c.z;s.history_R=R;s.model=o.model;s.length_m=ctx.length_m;s.time_scale_s=ctx.time_scale_s;
 s.solver_exitflag=c.exitflag;s.angular_cost_rad2=c.cost_rad2;
 s.status='CANDIDATE_ONLY_NOT_DISTANCE_VALIDATED';s.error_id='';s.error_message='';
 s.successes=s.successes+1;report.status=s.status;
catch ex
 % A failed refresh retains the previous cache until its ORIGINAL cutoff ages.
 s.failures=s.failures+1;s.status='REFRESH_FAILED';s.error_id=ex.identifier;s.error_message=ex.message;
 report.error_id=ex.identifier;report.error_message=ex.message;
end
report.elapsed_s=toc(started);
end
function a=candidate_summary(c)
a=struct('initial_range_m',c.initial_range_m,'cost_rad2',c.cost_rad2,'exitflag',c.exitflag, ...
 'error_id',c.error_id,'error_message',c.error_message,'constraints',c.constraints, ...
 'numerical_rank',c.numerical_rank,'condition_number',c.condition_number,'reference_range_m',NaN);
if ~isempty(c.geometry),a.reference_range_m=c.geometry.reference_range_m;end
end
