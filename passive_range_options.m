function options=passive_range_options()
% Historical engineering profile, NOT universal or independently validated.
options=struct('model','CV','apply_smoothing',false,'fit',passive_constrained_fit_config(), ...
 'smoothing',passive_measurement_smoothing_config());
end
