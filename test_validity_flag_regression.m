function result = test_validity_flag_regression()
%TEST_VALIDITY_FLAG_REGRESSION Regression checks for validity-token parsing.

valid_tokens = {'1', 1, true, '有效', '  有效  ', '"有效"'};
invalid_tokens = {'0', 0, false, '无效', '  无效  ', '"无效"'};
unknown_tokens = {'', '未知', 'NaN'};

for i = 1:numel(valid_tokens)
    assert(parse_validity_flag(valid_tokens{i}) == 1, ...
        'Expected a valid token at test index %d.', i);
end
for i = 1:numel(invalid_tokens)
    assert(parse_validity_flag(invalid_tokens{i}) == 0, ...
        'Expected an invalid token at test index %d.', i);
end
for i = 1:numel(unknown_tokens)
    assert(isnan(parse_validity_flag(unknown_tokens{i})), ...
        'Expected an unrecognized token at test index %d.', i);
end

assert(parse_validity_flag('无效') == 0, ...
    'Exact matching must prevent 无效 from being treated as 有效.');
assert(parse_validity_flag('0.5') == 1 && parse_validity_flag('0.49') == 0, ...
    'Historical numeric threshold compatibility changed unexpectedly.');

result = struct('passed', true, ...
    'valid_cases', numel(valid_tokens), ...
    'invalid_cases', numel(invalid_tokens), ...
    'unknown_cases', numel(unknown_tokens));
fprintf('有效性字段兼容回归通过: 有效=%d, 无效=%d, 未识别=%d。\n', ...
    result.valid_cases, result.invalid_cases, result.unknown_cases);
end
