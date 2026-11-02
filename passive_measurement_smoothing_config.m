function c=passive_measurement_smoothing_config()
% One additional fixed OFFLINE measurement smoothing pass; not truth-tuned.
c=struct('method','LOCAL_QUADRATIC_PHYSICAL_TIME','polynomial_degree',2,'half_window_s',.5, ...
 'minimum_points',5,'maximum_gap_s',.08,'passes',1,'is_causal',false);
end
