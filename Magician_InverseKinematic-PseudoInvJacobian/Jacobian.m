function [ J ] = Jacobian( FK )
%JACOBIAN Summary of this function goes here
%   Detailed explanation goes here
%%
Z0=[0;0;1];O=[0;0;0];O4=FK(1:3,16);
J1=cross(Z0,(O4-O));
%%
Z1=(FK(1:3,3));O1=FK(1:3,4);
J2=cross(Z1,(O4-O1));
%%
Z2=(FK(1:3,7));O2=FK(1:3,8);
J3=cross(Z2,(O4-O2));
%%
Z3=(FK(1:3,11));O3=FK(1:3,12);
J4=cross(Z3,(O4-O3));
%%
J=[J1 J2 J3 J4];
assignin('base','J',J);
end

