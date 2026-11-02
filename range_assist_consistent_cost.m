function [B,applied,reason]=range_assist_consistent_cost(A,cost,quality,available,reason,weight,unmatched_cost,angular_budget)
% Use the SAME feature set for every edge in one residual competition.
% Missing/failed models cause whole-component angular fallback. Explicit
% physically inconsistent trials have available=true and a bounded penalty.
if nargin<7,unmatched_cost=250;end
if nargin<8,angular_budget=Inf;end
B=A;applied=false(size(A));finite=isfinite(A);pending=any(finite,2);
while any(pending)
 rows=find(pending,1);previous=[];
 while ~isequal(rows,previous)
  previous=rows;cols=find(any(finite(rows,:),1));rows=find(any(finite(:,cols),2));
 end
 pending(rows)=false;mask=false(size(A));mask(rows,cols)=finite(rows,cols);
 ix=find(mask);
 if any(~available(ix))||any(~isfinite(cost(ix)))||any(~isfinite(quality(ix)))
  for j=reshape(ix(available(ix)),1,[]),reason{j}='COMPONENT_FALLBACK';end
  continue;
 end
 B(ix)=A(ix)+weight*quality(ix).*cost(ix);
 if isfinite(angular_budget)
  original=A(rows,cols);augmented=B(rows,cols);
  p0=solve_global_assignment(original,unmatched_cost);
  p1=solve_global_assignment(augmented,unmatched_cost);
  angular_loss=objective(original,p1,unmatched_cost)-objective(original,p0,unmatched_cost);
  if angular_loss>angular_budget+1e-10
   B(ix)=A(ix);
   for j=reshape(ix,1,[]),reason{j}='ANGULAR_PRIORITY_FALLBACK';end
   continue;
  end
 end
 applied(ix)=true;
end
end
function value=objective(A,pairs,unmatched)
value=(size(A,1)-size(pairs,1))*unmatched;
if ~isempty(pairs),value=value+sum(A(sub2ind(size(A),pairs(:,1),pairs(:,2))));end
end
