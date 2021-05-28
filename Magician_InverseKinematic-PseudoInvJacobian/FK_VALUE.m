%clear all;
%clc;

syms l0 l1 l2 l3 l4 l5 the1 the2 the3 the4;

% l0 = 30.6; l1 = 102.03; l2 = 177.5; l3 = 190; l4=81.3; l5=2.4;
% the1 = 90*pi/180; the2 = 118*pi/180; the3 = -157*pi/180;
% the4 = 2*pi-(the2 + the3);

%DEFINE TABLE  
DH = [  0         0      l1     the1;... 
        l0        pi/2    0     the2;...
        l2        0      0      the3;...
        l3        0      0      the4];

T_i = eye(4);
    
for i = 1:4,
    % EXTRACT DATA FROM DH TABLE
    a = DH(i,1); anp = DH(i,2); d = DH(i,3); the = DH(i,4);
    
    % FOMOGENOUS TRANSFORMATION MATRIX OF TWO CONSECUTIVE JOINTS
    % MA TRAN CHUYEN DOI DONG NHAT CUA 2 KHOP LIEN TIEP
    T_i_1_i =  [ cos(the)                -sin(the)               0          a;...
                (sin(the)*cos(anp))     (cos(the)*cos(anp))     -sin(anp)   -(sin(anp))*d;...
                (sin(the)*sin(anp))     (cos(the)*sin(anp))     cos(anp)    (cos(anp))*d;...
                0                       0                       0           1];
    T_i = T_i * T_i_1_i;
end

for i=1:3
    for j=1:3
        R(i,j) = T_i(i,j);
    end
end
R;
R_dot_the1 = diff(R,the1);
R_dot_the2 = diff(R,the2);
R_dot_the3 = diff(R,the3);
R_dot_the4 = diff(R,the4);

W_the1 = R*R_dot_the1
W_the2 = R*R_dot_the2
W_the3 = R*R_dot_the3

%POSITION OF END EFFECTOR W.R.T (3)
P_3_EE = [l4;0;l5;1];
P_0_EE = T_i * P_3_EE