function d=angular_distance_ambiguity(cost,referenceRange,relativeCostBand,absoluteCostBand,relativeSpreadLimit)
% Multi-start empirical ambiguity only; no confidence interval or certification.
if ~isvector(cost)||~isequal(size(cost),size(referenceRange))||any(cost(isfinite(cost))<0)||any(referenceRange(isfinite(referenceRange))<=0)
 error('BO3D:Angular:InvalidCandidates','Matching cost and positive reference-range vectors required.');
end
valid=isfinite(cost)&isfinite(referenceRange);idx=find(valid);
d=struct('best_index',[],'near_best_indices',[],'relative_range_spread',NaN,'ambiguity_detected',false,'status','NO_FINITE_CANDIDATE','range_is_validated',false);
if isempty(idx),return;end
[best,k]=min(cost(idx));d.best_index=idx(k);limit=best*(1+relativeCostBand)+absoluteCostBand;
near=find(valid&cost<=limit);rr=referenceRange(near);d.near_best_indices=near;
d.relative_range_spread=(max(rr)-min(rr))/referenceRange(d.best_index);
d.ambiguity_detected=numel(near)>=2&&d.relative_range_spread>relativeSpreadLimit;
if d.ambiguity_detected,d.status='DISTANCE_CONSTRAINT_INSUFFICIENT_MULTISTART';else,d.status='NO_AMBIGUITY_DETECTED_NOT_CERTIFICATION';end
end
