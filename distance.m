%����֮��ľ���

function [d] = distance(X, Y)
if length(Y) == 4
    d = sqrt((X(1) - Y(1))^2 + (X(3) - Y(3))^2);
end
if length(Y) == 2
    d = sqrt((X(1) - Y(1))^2 + (X(3) - Y(2))^2);
end
end
