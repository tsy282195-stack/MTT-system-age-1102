function info=export_range_diagnostics(est,track_ids,output_dir)
%EXPORT_RANGE_DIAGNOSTICS Raw-AE range history and selected candidate costs.
info=struct('available',false,'state_files',{{}},'association_files',{{}});
if ~isfield(est,'range_assist')||isempty(est.range_assist),return;end
has_state=cellfun(@(x)isstruct(x)&&isfield(x,'estimates')&&~isempty(x.estimates),est.range_assist);
if ~any(has_state),return;end
if ~exist(output_dir,'dir'),mkdir(output_dir);end
info.available=true;
for id=reshape(track_ids,1,[])
 state_file=fullfile(output_dir,sprintf('track_%g_range.csv',id));
 pair_file=fullfile(output_dir,sprintf('track_%g_range_associations.csv',id));
 f=fopen(state_file,'w','n','UTF-8');g=fopen(pair_file,'w','n','UTF-8');
 if f<0||g<0
  if f>=0,fclose(f);end;if g>=0,fclose(g);end
  error('RangeAssist:ExportFailed','Cannot write distance diagnostics.');
 end
 cleanup=onCleanup(@()close_files(f,g));
 fprintf(f,'event,time_s,data_cutoff_s,confirmed_s,initialized_s,range_m,range_std_m,quality,n_updates,available,status\n');
 fprintf(g,'event,time_s,meas_index,locked,applied,angular_cost,total_cost,range_cost,quality,range_std_m,predicted_range_m,candidate_range_m,delta_range_m,reason\n');
 legend=range_assist_reason();
 for k=1:numel(est.range_assist)
  d=est.range_assist{k};if isempty(d),continue;end
  if isfield(d,'estimates')&&~isempty(d.estimates)
   s=d.estimates;
   for j=find(s.track_id==id)
    fprintf(f,'%d,%.9f,%.9f,%.9f,%.9f,%.12g,%.12g,%.8g,%d,%d,%s\n', ...
     k,s.time_s(j),s.data_cutoff_s(j),s.confirmed_t(j),s.initialized_t(j), ...
     s.range_m(j),s.range_std_m(j),s.quality(j),s.n_updates(j),s.available(j),s.status{j});
   end
  end
  a=d.associations;
  if isempty(a)||~isfield(a,'quality'),continue;end
  t=NaN;if isfield(est,'filter_times')&&k<=numel(est.filter_times),t=est.filter_times(k);end
  for j=find(a.track_id==id)
   reason=legend{double(a.reason_code(j))+1};
   fprintf(g,'%d,%.9f,%d,%d,%d,%.12g,%.12g,%.12g,%.8g,%.12g,%.12g,%.12g,%.12g,%s\n', ...
    k,t,a.meas_index(j),a.locked(j),a.distance_applied(j),a.original_cost(j), ...
    a.final_cost(j),a.range_cost(j),a.quality(j),a.range_std_m(j), ...
    a.reference_range_m(j),a.candidate_range_m(j),a.delta_range_m(j),reason);
  end
 end
 clear cleanup;
 info.state_files{end+1}=state_file;info.association_files{end+1}=pair_file;
end
end
function close_files(f,g)
fclose(f);fclose(g);
end
