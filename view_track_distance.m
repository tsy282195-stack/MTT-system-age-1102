function info=view_track_distance(result_mat,truth_file,opts)
% 独立查看器：直接读结果与物理真值文件，绘制三维斜距及“估计-真值”。
% 不读取工作区变量，不调用本项目其他函数；所有辅助逻辑均在本文件内。
%VIEW_TRACK_DISTANCE Standalone 3-D slant-range comparison and signed error.
% No workspace variables or other project functions are read/called.
% All helpers are local to THIS file; only MATLAB built-in facilities used.
% auto prefers saved range_assist records; output_3d selects formal 3-D output.
%
%   view_track_distance
%   info=view_track_distance(result_mat,truth_file,struct('track_ids',[1009 1022]));
%   info=view_track_distance(result_mat,truth_file,struct( ...
%       'index_mode','truth','truth_ids',[192 193],'truth_id_type','raw'));
%
% Truth: physical truth.mat (truth/layout/cfg), long truth.csv, or wide
% truth_rae.txt. Result: review/full MAT containing est. No filtering runs.
% Frozen cached ranges are NOT treated as fresh estimates at later times.

%% 直接运行时的设置，与view_track.m相互独立。
here=fileparts(mfilename('fullpath'));
RESULT_MAT=fullfile(here,'track_reports','fusion_result.mat');
% 默认读取当前主程序结果；查看历史结果时，在此处或第一个函数参数中指定对应MAT。
TRUTH_FILE='';                 % 留空：从结果中记录的数据位置查找truth.mat/csv。
INDEX_MODE='track';            % 'track'=滤波编号；'truth'=真值编号。
TRACK_IDS=[1015 1025];          % []=全部有距离记录的航迹。
TRUTH_IDS=[62 63];                  % truth模式填写编号；[]=全部保存的真值。
TRUTH_ID_TYPE='raw';           % raw=原始编号，如192/193；instance=实例编号，如62/63。
DISTANCE_SOURCE='auto';        % auto优先反演估距；output_3d选择正式三维输出距离。
DISTANCE_UNIT='km';            % 'km'或'm'；info内数据始终使用米。
RELATIVE_TIME=true;            % true=距结果起点的秒数；false=绝对秒数。
TRACKS_PER_PAGE=4;
% opts.truth_map=[track_id raw_truth_id; ...] overrides automatic matching.
% opts.truth_time_offset_s overrides the truth clock origin (seconds).
% opts.time_range_s=[absolute_start absolute_end] limits the plotted window.
% opts.max_truth_gap_s (default .2) forbids interpolation over missing truth.
% opts.show_figures=false returns comparison data without opening figures.

if nargin<1||isempty(result_mat),result_mat=RESULT_MAT;end
if nargin<2||isempty(truth_file),truth_file=TRUTH_FILE;end
if nargin<3,opts=struct();end
if ~isstruct(opts)||~isscalar(opts),error('view_track_distance:Options','opts必须是标量结构体。');end
mode=lower(char(local_opt(opts,'index_mode',INDEX_MODE)));
source=lower(char(local_opt(opts,'distance_source',DISTANCE_SOURCE)));
unit=lower(char(local_opt(opts,'distance_unit',DISTANCE_UNIT)));
if ~any(strcmp(mode,{'track','truth'})),error('view_track_distance:IndexMode','index_mode必须为track或truth。');end
if ~any(strcmp(source,{'auto','range_assist','output_3d'})),error('view_track_distance:Source','distance_source必须为auto/range_assist/output_3d。');end
if ~any(strcmp(unit,{'m','km'})),error('view_track_distance:Unit','distance_unit必须为m或km。');end
result_mat=char(result_mat);
if ~isfile(result_mat),error('view_track_distance:ResultFile','找不到结果文件：%s',result_mat);end
inventory=whos('-file',result_mat);names={inventory.name};
wanted=intersect(names,{'est','cfg','trial','metrics_info','metrics','frame_times'},'stable');
S=load(result_mat,wanted{:});
if ~isfield(S,'est'),error('view_track_distance:MissingEstimate','结果文件不包含est。');end
cfg=local_opt(S,'cfg',local_opt(S,'trial',struct()));
metrics=local_opt(S,'metrics_info',local_opt(S,'metrics',struct()));
[D,source_used]=local_distance_records(S.est,source);
if isempty(D.id)
 error('view_track_distance:NoDistance', ...
  ['结果文件没有逐帧距离记录或三维输出距离。旧版batch结果不能还原完整距离历史；' ...
   '请指定新版review/full结果文件。']);
end
available=unique(D.id(isfinite(D.id)&D.id>0)).';
if isempty(available),error('view_track_distance:NoTrack','没有带公开航迹编号的距离记录。');end
[links,summary]=local_reference_links(S.est,metrics,available);
manual=local_opt(opts,'truth_map',zeros(0,2));
if ~isempty(manual)
 validateattributes(manual,{'numeric'},{'2d','real','finite','ncols',2});
 if numel(unique(manual(:,1)))~=size(manual,1)||any(manual(:)<=0)
  error('view_track_distance:TruthMap','truth_map须为唯一滤波编号到正真值编号的两列表。');
 end
 for i=1:numel(links)
  q=find(manual(:,1)==links(i).track_id,1);
  if ~isempty(q),links(i).raw_id=manual(q,2);links(i).method='手工对应';end
 end
end
requested_raw=[];
if strcmp(mode,'track')
 requested=TRACK_IDS;if isfield(opts,'track_ids'),requested=opts.track_ids;end
 local_validate_ids(requested,'track_ids');
 if isempty(requested),selected=available;else,selected=unique(requested(:).','stable');end
 missing=setdiff(selected,available);
 if ~isempty(missing),error('view_track_distance:NoTrack','距离记录中没有航迹%s；可用编号：%s',mat2str(missing),mat2str(available));end
else
 requested=local_opt(opts,'truth_ids',TRUTH_IDS);local_validate_ids(requested,'truth_ids');
 id_type=lower(char(local_opt(opts,'truth_id_type',TRUTH_ID_TYPE)));
 if strcmp(id_type,'instance')
  if isempty(summary)||~all(isfield(summary,{'instance_labels','key_raw_id'}))
   error('view_track_distance:NoInstanceMap','结果未保存实例到原始编号的对应表；请使用raw编号或truth_map。');
  end
  keys=summary.instance_labels(:);raw=summary.key_raw_id(:);
  valid=isfinite(raw)&raw>0;keys=keys(valid);raw=raw(valid);
  if isempty(requested),requested=keys.';end
  [hit,q]=ismember(requested,keys);
  if ~all(hit),error('view_track_distance:NoTruth','不存在的真值实例编号：%s',mat2str(requested(~hit)));end
  requested_raw=unique(raw(q).','stable');
 elseif strcmp(id_type,'raw')
  requested_raw=unique(requested(:).','stable');
  if isempty(requested_raw),requested_raw=unique([links.raw_id]);requested_raw=requested_raw(isfinite(requested_raw)&requested_raw>0);end
 else
  error('view_track_distance:TruthIdType','truth_id_type必须为raw或instance。');
 end
 selected=[];
 for i=1:numel(links)
  candidates=unique([links(i).raw_id,links(i).touched_raw]);
  if ~any(ismember(candidates,requested_raw)),continue;end
  selected(end+1)=links(i).track_id; %#ok<AGROW>
  if ~ismember(links(i).raw_id,requested_raw)
   hit=ismember(links(i).vote_raw,requested_raw);
   if any(hit)
    raw=links(i).vote_raw(hit);count=links(i).vote_count(hit);[~,q]=max(count);
    links(i).raw_id=raw(q);
   else
    links(i).raw_id=requested_raw(find(ismember(requested_raw,candidates),1));
   end
   links(i).method='所查询真值';
  end
 end
end
result_window=local_result_window(S,D);base_time=result_window(1);
window=local_opt(opts,'time_range_s',result_window);
validateattributes(window,{'numeric'},{'real','finite','vector','numel',2});
if window(2)<window(1),error('view_track_distance:TimeWindow','时间窗终点必须不早于起点。');end
chosen_links=links(ismember([links.track_id],selected));
raw_needed=unique([[chosen_links.raw_id],requested_raw]);
raw_needed=raw_needed(isfinite(raw_needed)&raw_needed>0);
if isempty(raw_needed),error('view_track_distance:NoReference','没有保存的真值对应关系；可用opts.truth_map指定[滤波编号 原始真值编号]。');end
if isempty(truth_file),truth_file=local_find_truth_file(cfg);end
truth_file=char(truth_file);
if ~isfile(truth_file),error('view_track_distance:TruthFile','找不到物理真值文件：%s',truth_file);end
[truth,truth_offset]=local_read_truth(truth_file,raw_needed,opts);
missing=setdiff(raw_needed,[truth.id]);
if ~isempty(missing),error('view_track_distance:NoTruth','真值文件缺少目标%s。请核对原始编号或使用truth_map。',mat2str(missing));end
gap=local_opt(opts,'max_truth_gap_s',.2);validateattributes(gap,{'numeric'},{'scalar','real','positive','finite'});
template=struct('track_id',NaN,'truth_id',NaN,'reference_method','', ...
 'time_s',[],'data_cutoff_s',[],'estimate_m',[],'range_std_m',[],'quality',[], ...
 'truth_m',[],'error_m',[],'valid',false(0,1),'n_valid',0, ...
 'rmse_m',NaN,'mean_error_m',NaN,'max_abs_error_m',NaN,'median_relative_error',NaN);
series=repmat(template,1,numel(selected));
fprintf('[距离结果] %s\n[物理真值] %s\n',result_mat,truth_file);
fprintf('[距离来源] %s；真值时间偏移=%.9f s；差值=估计距离-物理真值距离。\n',source_used,truth_offset);
for n=1:numel(selected)
 id=selected(n);link=links([links.track_id]==id);
 if ~isfinite(link.raw_id),error('view_track_distance:NoReference','Track %g没有真值对应，请填写truth_map。',id);end
 q=find(D.id==id&D.t>=window(1)&D.t<=window(2));
 [t,order]=sort(D.t(q));q=q(order);
 tk=truth([truth.id]==link.raw_id);reference=local_interpolate(tk,t,gap);
 fresh=D.available(q)&isfinite(D.range(q))&D.range(q)>0;
 if strcmp(source_used,'range_assist'),fresh=fresh&abs(D.cutoff(q)-t)<1e-6;end
 estimate=D.range(q);estimate(~fresh)=NaN;
 valid=fresh&isfinite(reference);err=estimate-reference;
 row=template;row.track_id=id;row.truth_id=link.raw_id;row.reference_method=link.method;
 row.time_s=t;row.data_cutoff_s=D.cutoff(q);row.estimate_m=estimate;
 row.range_std_m=D.std(q);row.quality=D.quality(q);row.truth_m=reference;
 row.error_m=err;row.valid=valid;row.n_valid=nnz(valid);
 if any(valid)
  row.rmse_m=sqrt(mean(err(valid).^2));row.mean_error_m=mean(err(valid));
  row.max_abs_error_m=max(abs(err(valid)));
  row.median_relative_error=median(abs(err(valid))./reference(valid));
 elseif any(fresh)
  error('view_track_distance:NoTimeOverlap', ...
   'Track %g的有效估距与真值无时间重叠；检查文件和truth_time_offset_s。',id);
 end
 series(n)=row;
 fprintf('[距离索引] Track %g -> 原始真值%g（%s），有效点=%d，RMSE=%.3f m，平均差值=%.3f m。\n', ...
  id,link.raw_id,link.method,row.n_valid,row.rmse_m,row.mean_error_m);
 truth_values=reference(isfinite(reference));
 if ~isempty(truth_values)
  fprintf('[真值距离] 起点=%.3f m，终点=%.3f m，最小=%.3f m，最大=%.3f m，变化范围=%.3f m。\n', ...
   truth_values(1),truth_values(end),min(truth_values),max(truth_values),max(truth_values)-min(truth_values));
 end
end
untracked=setdiff(requested_raw,[series.truth_id]);
figures=gobjects(0);
if local_opt(opts,'show_figures',true)
 relative=logical(local_opt(opts,'relative_time',RELATIVE_TIME));
 page_size=local_opt(opts,'tracks_per_page',TRACKS_PER_PAGE);
 validateattributes(page_size,{'numeric'},{'scalar','positive','integer','finite'});
 figures=local_plot(series,truth,untracked,window,gap,unit,relative,base_time,page_size);
end
info=struct('result_file',result_mat,'truth_file',truth_file,'distance_source',source_used, ...
 'available_track_ids',available,'selected_track_ids',selected,'truth_time_offset_s',truth_offset, ...
 'time_origin_s',base_time,'series',series,'truth_only_ids',untracked,'figures',figures);
end

function [D,source]=local_distance_records(est,requested)
D=struct('id',[],'event',[],'t',[],'cutoff',[],'range',[],'std',[],'quality',[],'available',false(0,1));
source=requested;
if ~strcmp(requested,'output_3d')&&isfield(est,'range_assist')
 counts=zeros(1,numel(est.range_assist));
 for k=1:numel(counts)
  e=est.range_assist{k};
  if isstruct(e)&&isfield(e,'estimates')&&isstruct(e.estimates)&&isfield(e.estimates,'track_id')
   counts(k)=numel(e.estimates.track_id);
  end
 end
 if any(counts)
  N=sum(counts);fields={'id','event','t','cutoff','range','std','quality'};
  for j=1:numel(fields),D.(fields{j})=nan(N,1);end
  D.available=false(N,1);p=0;
  for k=find(counts)
   e=est.range_assist{k}.estimates;n=counts(k);ix=p+(1:n);p=p+n;
   if ~all(isfield(e,{'time_s','range_m'}))||numel(e.time_s)~=n||numel(e.range_m)~=n
    error('view_track_distance:MalformedResult','第%d事件的距离字段长度不一致。',k);
   end
   D.id(ix)=e.track_id(:);D.event(ix)=k;D.t(ix)=e.time_s(:);D.range(ix)=e.range_m(:);
   D.cutoff(ix)=local_vector(e,'data_cutoff_s',n,e.time_s(:));
   D.std(ix)=local_vector(e,'range_std_m',n,nan(n,1));
   D.quality(ix)=local_vector(e,'quality',n,nan(n,1));
   D.available(ix)=logical(local_vector(e,'available',n,true(n,1)));
  end
  source='range_assist';return;
 end
end
if strcmp(requested,'range_assist'),return;end
if isfield(est,'output_history')&&isstruct(est.output_history)
 h=est.output_history;
 if all(isfield(h,{'id','t_sec','range_m','output_dim'}))
  keep=h.output_dim(:)==3;
  D.id=h.id(keep);D.t=h.t_sec(keep);D.range=h.range_m(keep);
  D.event=local_vector(h,'event_index',numel(h.id),nan(numel(h.id),1));D.event=D.event(keep);
 end
elseif isfield(est,'output')&&iscell(est.output)
 for k=1:numel(est.output)
  e=est.output{k};
  if isempty(e)||~all(isfield(e,{'id','t_sec','range_m','output_dim'})),continue;end
  e=e([e.output_dim]==3);n=numel(e);
  D.id=[D.id;reshape([e.id],[],1)];D.t=[D.t;reshape([e.t_sec],[],1)]; %#ok<AGROW>
  D.range=[D.range;reshape([e.range_m],[],1)];D.event=[D.event;k*ones(n,1)]; %#ok<AGROW>
 end
end
D.id=D.id(:);D.t=D.t(:);D.range=D.range(:);D.cutoff=D.t;
D.std=nan(size(D.t));D.quality=nan(size(D.t));D.available=isfinite(D.range)&D.range>0;
source='output_3d';
end

function [links,summary]=local_reference_links(est,metrics,ids)
template=struct('track_id',0,'raw_id',NaN,'method','', ...
 'touched_raw',[],'vote_raw',[],'vote_count',[]);
links=repmat(template,1,numel(ids));summary=[];
if isfield(metrics,'truth_targets'),summary=metrics.truth_targets;end
details=local_opt(metrics,'track_details',struct([]));
votes=cell(1,numel(ids));
if isfield(est,'assoc')
 for k=1:numel(est.assoc)
  a=est.assoc{k};if isempty(a)||~all(isfield(a,{'id','tid'})),continue;end
  n=min(numel(a.id),numel(a.tid));[hit,slot]=ismember(a.id(1:n),ids);
  for j=find(hit&isfinite(a.tid(1:n))&a.tid(1:n)>0)
   votes{slot(j)}(end+1)=a.tid(j); %#ok<AGROW>
  end
 end
end
for i=1:numel(ids)
 links(i).track_id=ids(i);
 if ~isempty(votes{i})
  [raw,~,q]=unique(votes{i});count=accumarray(q(:),1);
  links(i).touched_raw=raw(:).';links(i).vote_raw=raw(:).';links(i).vote_count=count(:).';
  [~,q]=max(count);links(i).raw_id=raw(q);links(i).method='已保存关联的多数编号';
 end
 if isempty(details)||~isfield(details,'track_id'),continue;end
 q=find([details.track_id]==ids(i),1);if isempty(q),continue;end
 detail=details(q);key=NaN;
 if isfield(detail,'primary')&&isfield(detail.primary,'matched_truth_id')
  key=detail.primary.matched_truth_id;
 elseif isfield(detail,'two_d')&&isfield(detail.two_d,'matched_truth_id')
  key=detail.two_d.matched_truth_id;
 end
 if isfinite(key)&&isstruct(summary)&&all(isfield(summary,{'instance_labels','key_raw_id'}))
  j=find(summary.instance_labels==key,1);
  if ~isempty(j)&&isfinite(summary.key_raw_id(j))&&summary.key_raw_id(j)>0
   links(i).raw_id=summary.key_raw_id(j);links(i).method='保存评价的主匹配';
  end
 end
end
end

function file=local_find_truth_file(cfg)
if ~isfield(cfg,'data_dir')||isempty(cfg.data_dir)
 error('view_track_distance:TruthFile','请在TRUTH_FILE或第二个函数参数中填写真值文件。');
end
folder=char(cfg.data_dir);
candidates={fullfile(fileparts(folder),'truth_data','truth.mat'), ...
 fullfile(fileparts(folder),'truth_data','truth.csv'),fullfile(folder,'truth.mat'),fullfile(folder,'truth.csv')};
for k=1:numel(candidates),if isfile(candidates{k}),file=candidates{k};return;end;end
error('view_track_distance:TruthFile','未在结果数据目录附近找到物理真值，请填写TRUTH_FILE。');
end

function [series,offset]=local_read_truth(file,ids,opts)
[~,~,ext]=fileparts(file);offset=local_opt(opts,'truth_time_offset_s',[]);
if ~isempty(offset),validateattributes(offset,{'numeric'},{'scalar','real','finite'});end
if strcmpi(ext,'.mat')
 T=load(file);truth=local_opt(T,'truth',T);
 if ~all(isfield(truth,{'time_s','range_m'}))
  error('view_track_distance:TruthFormat','真值MAT需要truth.time_s、truth.range_m及目标编号。');
 end
 if isfield(truth,'target_ids'),target_ids=truth.target_ids(:).';
 elseif isfield(truth,'target_id'),target_ids=truth.target_id(:).';
 elseif isfield(T,'layout')&&isfield(T.layout,'targets')&&isfield(T.layout.targets,'target_id')
  target_ids=reshape([T.layout.targets.target_id],1,[]);
 else,error('view_track_distance:TruthFormat','MAT缺少target_ids或layout.targets.target_id，不能按矩阵列号猜测编号。');end
 if isempty(offset)
  if isfield(T,'cfg')&&isfield(T.cfg,'start_clock_s'),offset=T.cfg.start_clock_s;else,offset=0;end
 end
 offset=double(offset);t=double(truth.time_s(:))+offset;r=double(truth.range_m);
 if size(r,1)~=numel(t)||size(r,2)~=numel(target_ids)
  error('view_track_distance:TruthFormat','真值距离矩阵应为[时间点数×目标数]。');
 end
 series=struct('id',{},'t',{},'r',{});
 for id=ids
  j=find(target_ids==id,1);if isempty(j),continue;end
  keep=isfinite(r(:,j))&r(:,j)>0;
  if isfield(truth,'alive')&&isequal(size(truth.alive),size(r)),keep=keep&logical(truth.alive(:,j));end
  series(end+1)=local_truth_series(id,t(keep),r(keep,j)); %#ok<AGROW>
 end
else
 imp=detectImportOptions(file,'VariableNamingRule','preserve','TextType','string');names=imp.VariableNames;
 if all(ismember({'target_id','time_s'},names))&&any(ismember({'true_range_m','range_m'},names))
  range_name='true_range_m';if ~ismember(range_name,names),range_name='range_m';end
  selected={'target_id','time_s',range_name};
  if ismember('time',names),selected{end+1}='time';imp=setvartype(imp,'time','string');end
  imp.SelectedVariableNames=selected;table=readtable(file,imp);
  if isempty(offset)
   offset=0;
   if ismember('time',selected)&&height(table)>0
    offset=local_clock_seconds(table.time(1))-table.time_s(1);
   end
  end
  offset=double(offset);raw=double(table.target_id);t=double(table.time_s)+offset;r=double(table.(range_name));
 else
  id_names=names(~cellfun('isempty',regexp(names,'^target_id_\d+$','once')));
  if isempty(id_names)||~ismember('time',names)
   error('view_track_distance:TruthFormat','CSV需target_id/time_s/true_range_m，宽表需time/target_id_N/range_m_N。');
  end
  range_names=regexprep(id_names,'^target_id_','range_m_');
  if ~all(ismember(range_names,names)),error('view_track_distance:TruthFormat','宽表缺少对应range_m_N列。');end
  valid_names=regexprep(id_names,'^target_id_','range_valid_');valid_names=valid_names(ismember(valid_names,names));
  imp=setvartype(imp,'time','string');
  if ~isempty(valid_names),imp=setvartype(imp,valid_names,'string');end
  imp.SelectedVariableNames=[{'time'},id_names,range_names,valid_names];
  table=readtable(file,imp);t0=local_clock_seconds(table.time);
  t0=t0+86400*cumsum([0;diff(t0)<-43200]);
  if isempty(offset),offset=0;end
  offset=double(offset);raw=table{:,id_names};r=table{:,range_names};t=repmat(t0+offset,1,numel(id_names));
  for k=1:numel(id_names)
   flag=regexprep(id_names{k},'^target_id_','range_valid_');
   if ismember(flag,valid_names)
    valid=ismember(lower(strtrim(table.(flag))),["有效","1","true","valid"]);r(~valid,k)=NaN;
   end
  end
  raw=raw(:);r=r(:);t=t(:);
 end
 series=struct('id',{},'t',{},'r',{});
 for id=ids
  keep=raw==id&isfinite(t)&isfinite(r)&r>0;
  if any(keep),series(end+1)=local_truth_series(id,t(keep),r(keep));end %#ok<AGROW>
 end
end
validateattributes(offset,{'numeric'},{'scalar','real','finite'});
end

function s=local_truth_series(id,t,r)
[t,q]=sort(t(:));r=r(q);[u,first,group]=unique(t);
if numel(u)<numel(t)
 spread=accumarray(group,r,[],@max)-accumarray(group,r,[],@min);
 if any(spread>1e-3),error('view_track_distance:AmbiguousTruth','真值编号%g同一时间有多个距离，无法唯一匹配。',id);end
end
s=struct('id',id,'t',u,'r',r(first));
end

function r=local_interpolate(s,t,max_gap)
r=nan(size(t));
if isempty(s.t),return;end
if isscalar(s.t),r(abs(t-s.t)<1e-6)=s.r;return;end
r=interp1(s.t,s.r,t,'linear',NaN);
lo=interp1(s.t,(1:numel(s.t))',t,'previous',NaN);
hi=interp1(s.t,(1:numel(s.t))',t,'next',NaN);
good=isfinite(lo)&isfinite(hi);bad=false(size(t));
bad(good)=s.t(hi(good))-s.t(lo(good))>max_gap;
r(bad)=NaN;
% Clock/relative-time conversion may differ by a few floating-point ulps.
nearest=interp1(s.t,(1:numel(s.t))',t,'nearest','extrap');
q=find(isfinite(nearest));tol=max(1e-6,64*eps(max(abs(s.t))));
exact=q(abs(t(q)-s.t(nearest(q)))<=tol);
r(exact)=s.r(nearest(exact));
end

function figures=local_plot(series,truth,untracked,window,gap,unit,relative,origin,page_size)
scale=1;if strcmp(unit,'km'),scale=1000;end
shift=0;if relative,shift=origin;end
xlabel_text='时间 (s)';if relative,xlabel_text='距结果起点时间 (s)';end
figures=gobjects(0);N=numel(series)+numel(untracked);
for start=1:page_size:N
 slots=start:min(start+page_size-1,N);rows=numel(slots);
 comparison=figure('Name','三维估计距离与物理真值对比','Color','w','Position',[70,60,1100,min(1100,max(420,260*rows))]);
 layout1=tiledlayout(comparison,rows,1,'TileSpacing','compact','Padding','compact');
 difference=figure('Name','三维估计距离减真值','Color','w','Position',[110,85,1100,min(1100,max(420,260*rows))]);
 layout2=tiledlayout(difference,rows,1,'TileSpacing','compact','Padding','compact');
 figures(end+1:end+2)=[comparison,difference]; %#ok<AGROW>
 for j=1:rows
  index=slots(j);ax1=nexttile(layout1);hold(ax1,'on');grid(ax1,'on');box(ax1,'on');
  ax2=nexttile(layout2);hold(ax2,'on');grid(ax2,'on');box(ax2,'on');
  if index<=numel(series)
   s=series(index);tk=truth([truth.id]==s.truth_id);
   keep=tk.t>=window(1)&tk.t<=window(2);
   [tt,rr]=local_break_gaps(tk.t(keep),tk.r(keep)/scale,gap);
   plot(ax1,tt-shift,rr,'k--','LineWidth',1.3,'DisplayName',sprintf('物理真值 %g',s.truth_id));
   [tt,rr]=local_break_gaps(s.time_s,s.estimate_m/scale,gap);
   plot(ax1,tt-shift,rr,'b-','LineWidth',1.5,'DisplayName',sprintf('Track %g 距离估计',s.track_id));
   [tt,rr]=local_break_gaps(s.time_s,s.error_m/scale,gap);
   plot(ax2,tt-shift,rr,'b-','LineWidth',1.3,'DisplayName','估计-真值');
   title(ax1,sprintf('Track %g / 原始真值 %g',s.track_id,s.truth_id));
   title(ax2,sprintf('Track %g：有效点 %d，RMSE %.3f %s',s.track_id,s.n_valid,s.rmse_m/scale,unit));
  else
   id=untracked(index-numel(series));tk=truth([truth.id]==id);keep=tk.t>=window(1)&tk.t<=window(2);
   [tt,rr]=local_break_gaps(tk.t(keep),tk.r(keep)/scale,gap);
   plot(ax1,tt-shift,rr,'k--','DisplayName',sprintf('物理真值 %g',id));
   title(ax1,sprintf('真值 %g：没有对应距离航迹',id));title(ax2,'没有距离估计，不计算差值');
  end
  yline(ax2,0,'k:','HandleVisibility','off');
  ylabel(ax1,['三维斜距 (',unit,')']);ylabel(ax2,['估计-真值 (',unit,')']);
  xlabel(ax1,xlabel_text);xlabel(ax2,xlabel_text);
  if window(2)>window(1),xlim(ax1,window-shift);xlim(ax2,window-shift);end
  legend(ax1,'Location','best');
 end
 title(layout1,'三维斜距估计与独立物理真值');title(layout2,'三维斜距差值（估计-真值）');
end
end

function [t,y]=local_break_gaps(t,y,gap)
t=t(:);y=y(:);q=find(diff(t)>gap);
for j=numel(q):-1:1
 k=q(j);t=[t(1:k);NaN;t(k+1:end)];y=[y(1:k);NaN;y(k+1:end)]; %#ok<AGROW>
end
end
function window=local_result_window(S,D)
if isfield(S,'frame_times')&&~isempty(S.frame_times),t=S.frame_times;
elseif isfield(S.est,'filter_times')&&~isempty(S.est.filter_times),t=S.est.filter_times;
else,t=D.t;end
t=t(isfinite(t));window=[min(t),max(t)];
end
function value=local_clock_seconds(text)
text=string(text);value=nan(size(text));
for k=1:numel(text)
 parts=sscanf(char(text(k)),'%f:%f:%f');
 if numel(parts)~=3,error('view_track_distance:TruthTime','无法解析真值时钟：%s',text(k));end
 value(k)=parts(1)*3600+parts(2)*60+parts(3);
end
end
function v=local_vector(s,name,n,fallback)
v=fallback(:);
if isfield(s,name)
 if numel(s.(name))~=n,error('view_track_distance:MalformedResult','字段%s长度不一致。',name);end
 v=double(s.(name)(:));
end
end
function value=local_opt(s,name,fallback)
if isstruct(s)&&isfield(s,name)&&~isempty(s.(name)),value=s.(name);else,value=fallback;end
end
function local_validate_ids(ids,name)
if ~isempty(ids),validateattributes(ids,{'numeric'},{'vector','real','finite','positive'},mfilename,name);end
end
