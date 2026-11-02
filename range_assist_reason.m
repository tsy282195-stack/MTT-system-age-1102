function value=range_assist_reason(reason)
% Numeric reason codes are zero-based indices into this saved legend.
legend={'NOT_APPLIED','STABLE_LOCK_BYPASS','LOCAL_LINEAR_CONTINUATION', ...
 'NO_CACHE','CACHE_NOT_PRIOR','CACHE_EXPIRED','INVALID_GEOMETRY', ...
 'CONTINUATION_INFEASIBLE','OTHER_NUMERICAL_FAILURE', ...
 'RECURSIVE_CONTINUATION','MODEL_INCONSISTENT','COMPONENT_FALLBACK','ANGULAR_PRIORITY_FALLBACK'};
if nargin==0,value=legend;return;end
k=find(strcmp(reason,legend),1);if isempty(k),k=9;end
value=uint8(k-1);
end
