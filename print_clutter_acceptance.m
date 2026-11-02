function print_clutter_acceptance(c, label, fid)
%PRINT_CLUTTER_ACCEPTANCE Shared console/TXT formatting, including missing data.
if nargin<2,label='杂波接纳';end
if nargin<3,fid=1;end
if isfield(c,'definition_enabled') && ~c.definition_enabled
    fprintf(fid,'  [%s] 已关闭按编号定义杂波；杂波接纳比例不适用。\n',label);
    return;
end
fprintf(fid,'  [%s] 杂波输入=%d',label,c.n_input);
if ~c.available
    fprintf(fid,'；起始/更新/总接纳不可计算（%d条记录缺少实际确认历史，需重新滤波）。\n',c.n_unresolved);
elseif c.n_input==0
    fprintf(fid,'；成功起始=0，确认后更新=0，总接纳=0；比例无样本。\n');
else
    fprintf(fid,'；成功起始=%d (%.4f%%)，确认后更新=%d (%.4f%%)，总接纳=%d (%.4f%%)。\n', ...
        c.n_init_success,100*c.init_acceptance_rate,c.n_update,100*c.update_acceptance_rate, ...
        c.n_accepted,100*c.acceptance_rate);
end
end
