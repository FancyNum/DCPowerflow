function [theta1,P_branch,M,Z,slackbus] = DCpowerflow(mpc)
[n,~] = size(mpc.bus);
[L,~] = size(mpc.branch);
A = zeros(n);%�ڵ㵼�ɾ���
for i = 1:L
    p = mpc.branch(i,1);q = mpc.branch(i,2);
    A(p,q) = -1/mpc.branch(i,4);
    A(q,p) = A(p,q);
end
for i = 1:n
    A(i,i) = -sum(A(i,:));
end
slackbus = find(mpc.bus(:,2)==3);
A(slackbus,:) = [];
A(:,slackbus) = [];

Z = inv(A);   %%�ڵ��迹����

P = zeros(n,1);%�����ڵ��ע�빦��
[x_gen,~] = size(mpc.gen);
for i = 1:x_gen
    P(mpc.gen(i,1)) = mpc.gen(i,2);
end
P = P - mpc.bus(:,3);
P(slackbus,:) = [];
theta = A\P;

index = (1:n)';index(slackbus) = [];
theta1 = zeros(n,1);%��ƽ������Ǽ��룬�γ����е�����ʸ��
[xx,~] = size(index);
for i = 1:xx
    theta1(index(i)) = theta(i);
end
theta1(slackbus) = 0;

P_branch = zeros(L,1);%֧·��������
for i = 1:L
    p = mpc.branch(i,1);q = mpc.branch(i,2);
    xx = theta1(p) - theta1(q);
    P_branch(i) = xx/mpc.branch(i,4);
end

M = zeros(n,L);%�ڵ�֧·��������
for i = 1:L
    p = mpc.branch(i,1);q = mpc.branch(i,2);
    if theta1(p) > theta1(q)
        M(p,i) = 1;M(q,i) = -1;
        if p == slackbus
            M(p,i) = 0;
        elseif q == slackbus
            M(q,i) = 0;        
        end
    else
        M(p,i) = -1;M(q,i) = 1;
        if p == slackbus
            M(p,i) = 0;
        elseif q == slackbus
            M(q,i) = 0; 
        end
    end
end
