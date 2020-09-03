function main
clear;
T = 0.5;%����ʱ��
M = 20;%��������
delta_w = 1e-4;%���������������������Խ��,Ŀ�����еĻ�����Խ�󣬹켣Խ���
Q = delta_w * diag([0.5, 1, 0.5, 1]);%��������������
R = 2;%�۲���������
F = [1, T, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, T;
     0, 0, 0, 1];%״̬ת�ƾ���
PFrms = zeros(1, M);
%%%%%%%%%%ϵͳ��ʼ��%%%%%%%%%%%%%
Length = 100; %Ŀ���˶��ĳ��ؿռ�
Width = 100;
%�۲�վ��λ���漴����
Node.x = Width * rand;
Node.y = Length * rand;
%%%%%%%%%%%Ŀ����˶��켣%%%%%%%%%%%
X = zeros(4, M);%Ŀ��״̬
Z = zeros(1, M);%�۲�����
w = randn(4, M);%��������
v = randn(1, M);%�۲�����
X(:, 1) = [1, Length/M, 20, 60/M]';%��ʼ��Ŀ��״̬
state0 = X(:, 1);%���Ƴ�ʼ��
for t = 2:M
    X(:, t) = F * X(:, t-1) + sqrtm(Q) * w(:, t);%Ŀ����ʵ�켣
end
%ģ��Ŀ���˶����̣��۲�վ��Ŀ��۲��ȡ��������
for t = 1:M
    x0 = Node.x;
    y0 = Node.y;
    Z(1, t) = feval('hfun', X(:, t), x0, y0) + sqrtm(R) * v(1, t);%Z(k+1) = h(X(k)) + v(k)
end
%�������
canshu.T = T;
canshu.M = M;
canshu.Q = Q;
canshu.R = R;
canshu.F = F;
canshu.state0 = state0;
%�˲�
[Xpf, Tpf] = PF(Z, Node, canshu);
%RMS�Ƚ�ͼ
for t = 1:M
    PFrms(1, t) = distance(X(:, t), Xpf(:, t));
end
%��ͼ
figure
hold on
box on
%�۲�վλ��
h1 = plot(Node.x, Node.y, 'ro', 'MarkerFaceColor', 'b');
%Ŀ����ʵ�켣
h2 = plot(X(1, :), X(3, :), '--m.', 'MarkerEdgeColor', 'm');
%�˲��㷨�켣
h3 = plot(Xpf(1, :), Xpf(3, :), '-k*', 'MarkerEdgeColor','b');
xlabel('X/m');
ylabel('Y/m');
legend([h1, h2, h3], '�۲�վλ��', 'Ŀ����ʵ�켣', 'PF�㷨�켣');
hold off
%RMS �������ͼ
figure
hold on 
box on
plot(PFrms(1, :), '-k.', 'MarkerEdgeColor', 'm');
xlabel('time/s');
ylabel('error/m');
legend('RMS�������');
title(['RMSE, q = ', num2str(delta_w)])
hold off
%ʵʱ�ԱȽ�ͼ
figure
hold on
box on
plot(Tpf(1, :), '-k.', 'MarkerEdgeColor', 'm');
xlabel('step');
ylabel('time/s');
legend('ÿ������������PF����ʱ��');
title('Comparison of Realtime')
hold off