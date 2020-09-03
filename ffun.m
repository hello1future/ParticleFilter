%��Ŀ��λ�ú���
%����������۲�վһ�ι۲�ֵx���۲�վ��λ�ã�x0��y0��
%���������Ŀ���λ����Ϣ
function [y] = ffun(x, x0, y0)
if nargin < 3
    error('Not enough input arguments.');
end
[row, col] = size(x);
if row ~= 4 || col ~= 1
    error('Input arguments error!');
end
y = zeros(2, 1);
y(1) = x(1) * cos(x(2)) + x0;
y(2) = x(1) * sin(x(2)) + y0;
end

