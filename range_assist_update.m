function [s,report]=range_assist_update(s,id,birth_event,t,ae_deg,sensor,o,R_deg2)
% Commit one accepted observation AFTER assignment. Trials never call this.
if strcmp(o.mode,'batch')
 [s,report]=range_assist_batch_update(s,id,birth_event,t,ae_deg,sensor,o);return;
end
if nargin<8,R_deg2=diag([0.5,0.1].^2);end
[s,report]=range_assist_recursive('update',s,id,birth_event,t,ae_deg,sensor,o,R_deg2);
end
