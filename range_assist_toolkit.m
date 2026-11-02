function range_assist_toolkit(verify,include_smoothing)
% Resolve the mathematical/API functions beside the fusion entry points.
persistent initialized
if nargin<1,verify=false;end
if nargin<2,include_smoothing=false;end
if ~isempty(initialized)&&~verify&&~include_smoothing,return;end
root=fileparts(mfilename('fullpath'));
if isempty(initialized)
 addpath(root);
end
for name={'make_passive_range_input','passive_range_options','passive_range_estimate', ...
        'passive_constrained_fit_config','passive_measurement_smoothing_config', ...
        'angular_motion_context','angular_motion_residual','angular_distance_ambiguity', ...
        'fit_constrained_angular_motion','motion_kinematic_constraints'}
 if ~strcmpi(which(name{1}),fullfile(root,[name{1} '.m']))
  error('RangeAssist:ToolkitResolution','Wrong toolkit function: %s.',name{1});end
end
if include_smoothing
 optional_root=fullfile(root,'third_party','passive_range_toolkit','src','smoothing');
 addpath(optional_root);
 if ~strcmpi(which('smooth_passive_angles'),fullfile(optional_root,'smooth_passive_angles.m'))
  error('RangeAssist:ToolkitResolution','Wrong toolkit function: smooth_passive_angles.');end
end
initialized=true;
end
