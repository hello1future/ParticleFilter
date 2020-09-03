function [Xout, Tpf] = PF(Z, Node, canshu)
T = canshu.T;
M = canshu.M;
Q = canshu.Q;
R = canshu.R;
F = canshu.F;
state0 = canshu.state0;
%%%%%%%%%%%%%%初始化滤波器%%%%%%%%%%%%%%%
N = 100;%粒子数
zPred = zeros(1, N);
Weight = zeros(1, N);
xparticlePred = zeros(4, N);
Xout = zeros(4, M);
Xout(:, 1) = state0;
Tpf = zeros(1, M);
xparticle = zeros(4, N);
for j = 1:N
    xparticle(:, j) = state0; %粒子集初始化
end
Xpf = zeros(4, N);
Xpf(:, 1) = state0;
for t = 2:M
    tic;
    XX = 0;
    x0 = Node.x;
    y0 = Node.y;
    %采样
    for k = 1:N
        xparticlePred(:, k) = feval('sfun', xparticle(:, k), T, F) + 5 * sqrtm(Q) * randn(4, 1);
    end
    %重要性权值计算
    for k = 1:N
        zPred(1, k) = feval('hfun', xparticlePred(:, k), x0, y0);
        z1 = Z(1, t) - zPred(1, k);
        Weight(1, k) = inv(sqrt(2 * pi * det(R))) * exp(-.5 * (z1)' * inv(R) * (z1)) + 1e-99;%权值计算
    end
    %归一化权重
    Weight(1, :) = Weight(1, :)./sum(Weight(1, :));
    %重新采样
    outIndex = randomR(1:N, Weight(1, :)'); %随机重采样
    xparticle = xparticlePred(:, outIndex); %获取新采样值
    target = [mean(xparticle(1, :)), mean(xparticle(2, :)), ...
        mean(xparticle(3, :)), mean(xparticle(4, :))]';
    Xout(:, t) = target;
    Tpf(1, t) = toc;
end
end