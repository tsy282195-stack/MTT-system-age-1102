function out=map_range_diagnostics(in,assoc,residual_map)
% Preserve branch identity and translate local measurement slots/public IDs.
out=in;
for k=1:numel(in)
 d=in{k};if isempty(d),continue;end
 a=assoc{k};
 if isfield(d.associations,'reason_code')
  r=d.associations;local=r.meas_index;
  valid=local>=1&local<=numel(residual_map{k});
  r.meas_index(valid)=residual_map{k}(local(valid));
  [hit,q]=ismember([r.branch_2d_id(:),r.meas_index(:)], ...
      [a.branch_2d_id(:),a.meas_index(:)],'rows');
  r.track_id=nan(size(local));r.track_id(hit)=a.id(q(hit));d.associations=r;
 else
  % Readability of first implementation's saved evidence is retained.
  for j=1:numel(d.associations)
   r=d.associations(j);branch=r.branch_2d_id;local=r.meas_index;
   if local>=1&&local<=numel(residual_map{k}),r.meas_index=residual_map{k}(local);end
   q=find(a.branch_2d_id==branch & a.meas_index==r.meas_index,1);
   r.track_id=NaN;if ~isempty(q),r.track_id=a.id(q);end
   d.associations(j)=r;
  end
 end
 for j=1:numel(d.updates)
  branch=d.updates(j).track_id;
  q=find(a.branch_2d_id==branch,1);
  d.updates(j).branch_2d_id=branch;
  d.updates(j).track_id=NaN;if ~isempty(q),d.updates(j).track_id=a.id(q);end
 end
 if ~isempty(d.edges),d.edges.source_meas_indices=residual_map{k};end
 if isfield(d,'estimates')&&~isempty(d.estimates)
  r=d.estimates;[hit,q]=ismember(r.branch_2d_id,a.branch_2d_id);
  r.track_id=nan(size(r.branch_2d_id));r.track_id(hit)=a.id(q(hit));d.estimates=r;
 end
 out{k}=d;
end
end
