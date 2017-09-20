function [ a,b,c,d ] = get4Corners( x,y,theta,L )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
R = [cos(theta),-sin(theta);sin(theta),cos(theta)];
point1 = [x+0.5*L;y-0.5*L];
point2 = [x+0.5*L;y+0.5*L];
point3 = [x-0.5*L;y+0.5*L];
point4 = [x-0.5*L;y-0.5*L];

point1 = point1 - [x;y];
point2 = point2 - [x;y];
point3 = point3 - [x;y];
point4 = point4 - [x;y];

a = R*point1 + [x,y]';
b = R*point2 + [x,y]';
c = R*point3 + [x,y]';
d = R*point4 + [x,y]';

end

