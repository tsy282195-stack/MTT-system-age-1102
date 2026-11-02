function o=range_assist_options(cfg)
%RANGE_ASSIST_OPTIONS Explicit, validated experimental association feature.
o=struct('enabled',false,'weight',.1,'interval_s',2,'window_s',20, ...
 'min_span_s',10,'sample_interval_s',.05,'smoothing_half_window_s',.5, ...
 'model','CV','scale_m',20000,'max_age_s',6,'max_samples',501, ...
 'max_iterations',4000,'max_function_evaluations',12000, ...
 'initial_ranges_m',[70000,100000,130000,160000], ...
 'bounds',struct('speed_min_mps',200,'speed_max_mps',700,'acceleration_max_mps2',30), ...
 'diagnostic_level','summary','diagnostic_event_indices',[], ...
 'mode','batch','recursive_iterations',2,'process_accel_std_mps2',2, ...
 'jump_az_deg',0.5,'jump_el_deg',0.15,'gap_reset_s',0.5, ...
 'cost_cap',9,'initial_position_std_fraction',0.5,'initial_velocity_std_mps',350, ...
 'angular_tie_budget',0.01);
names=fieldnames(o);
for k=1:numel(names)
 f=['joint_range_assist_' names{k}];if isfield(cfg,f),o.(names{k})=cfg.(f);end
end
if ~(isscalar(o.enabled)&&(islogical(o.enabled)||(isnumeric(o.enabled)&&isreal(o.enabled)&&ismember(o.enabled,[0,1]))))
 error('RangeAssist:InvalidConfig','enabled must be a scalar flag.');end
o.enabled=logical(o.enabled);
if ~isa(o.weight,'double')||~isreal(o.weight)||~isscalar(o.weight)||~isfinite(o.weight)||o.weight<0|| ...
 (strcmp(o.mode,'batch')&&o.weight>1)
 error('RangeAssist:InvalidConfig','weight must be nonnegative; batch weights must also be <=1.');end
for f={'interval_s','window_s','min_span_s','sample_interval_s','smoothing_half_window_s','scale_m','max_age_s','max_samples','max_iterations','max_function_evaluations', ...
 'recursive_iterations','process_accel_std_mps2','jump_az_deg','jump_el_deg','gap_reset_s','cost_cap','initial_position_std_fraction','initial_velocity_std_mps','angular_tie_budget'}
 x=o.(f{1});if ~isa(x,'double')||~isreal(x)||~isscalar(x)||~isfinite(x)||x<=0
  error('RangeAssist:InvalidConfig','%s must be a positive finite double.',f{1});end
end
for f={'max_samples','max_iterations','max_function_evaluations','recursive_iterations'}
 if fix(o.(f{1}))~=o.(f{1}),error('RangeAssist:InvalidConfig','%s must be integral.',f{1});end
end
if ~ischar(o.mode)||~any(strcmp(o.mode,{'batch','recursive'}))
 error('RangeAssist:InvalidConfig','mode must be batch or recursive.');end
if o.window_s<o.min_span_s||o.max_samples<ceil(o.min_span_s/o.sample_interval_s)+1|| ...
 (strcmp(o.mode,'batch') && (o.smoothing_half_window_s<4*o.sample_interval_s||o.sample_interval_s>.08||o.max_age_s<o.interval_s))
 error('RangeAssist:InvalidConfig','Inconsistent window, sampling, support, capacity or cache age.');end
if strcmp(o.mode,'recursive') && ~strcmp(o.model,'CV')
 error('RangeAssist:InvalidConfig','Recursive distance currently requires CV.');end
if ~ischar(o.model)||~ismember(o.model,{'CV','CA'})||~ischar(o.diagnostic_level)|| ...
 ~ismember(o.diagnostic_level,{'summary','edges','full'})
 error('RangeAssist:InvalidConfig','Explicit CV/CA and summary/edges/full required.');end
x=o.initial_ranges_m;
if ~isa(x,'double')||~isreal(x)||~isvector(x)||isempty(x)||any(~isfinite(x))||any(x<=0)
 error('RangeAssist:InvalidConfig','Positive finite initial ranges required.');end
b=o.bounds;
if ~isstruct(b)||~isscalar(b)||~isequal(sort(fieldnames(b)),sort({'speed_min_mps';'speed_max_mps';'acceleration_max_mps2'}))
 error('RangeAssist:InvalidConfig','Exact speed/acceleration norm bounds required.');end
x=[b.speed_min_mps,b.speed_max_mps,b.acceleration_max_mps2];
if ~isa(x,'double')||~isreal(x)||numel(x)~=3||any(~isfinite(x))||x(1)<0||x(2)<=x(1)||x(3)<0
 error('RangeAssist:InvalidConfig','Invalid speed/acceleration norm bounds.');end
x=o.diagnostic_event_indices;
if ~isa(x,'double')||~isreal(x)||(~isempty(x)&&~isvector(x))||any(~isfinite(x))||any(x<1|fix(x)~=x)
 error('RangeAssist:InvalidConfig','Diagnostic event indices must be positive integers.');end
end
