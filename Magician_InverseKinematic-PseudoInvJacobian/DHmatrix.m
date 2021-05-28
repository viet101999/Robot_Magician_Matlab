function [Mdh] = DHmatrix(theta,d,a,alpha)
%DHMATRIX Summary of this function goes here
%   Detailed explanation goes here
%   inputannya DH parameter yaa
Mdh=[cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
     sind(theta) cosd(theta)*cosd(alpha)  -cosd(theta)*sind(alpha) a*sind(theta);
     0,sind(alpha),cosd(alpha),d;
     0,0,0,1];
end

