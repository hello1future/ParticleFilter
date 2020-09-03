%系统转移函数
function [y] = sfun(x, T, F)
if nargin < 2
    error('Not enough input arguments.');
end
y = F * x;
end

