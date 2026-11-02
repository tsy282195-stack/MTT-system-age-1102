function [cost,d]=range_assist_pair_cost(s,t,sensor,predicted_ae_deg,candidate_ae_deg,o,R_deg2)
% Recursive input is raw AE. Every candidate uses the same immutable prior.
if strcmp(o.mode,'batch')
 [cost,d]=range_assist_batch_pair_cost(s,t,sensor,predicted_ae_deg,candidate_ae_deg,o);return;
end
if nargin<7,R_deg2=diag([0.5,0.1].^2);end
prior=range_assist_recursive('predict',s,t,sensor,o);
[cost,d]=range_assist_recursive('candidate',prior,candidate_ae_deg,R_deg2,o);
end
