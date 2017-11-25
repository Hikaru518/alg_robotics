% 
% clear all
% clc
%
%% setting parameters
n = 3;
l = [1,1,1];
data = importdata('n=path.txt');

%% setting obstacles
xlist = [-0.5,-0.5,0,0,-0.5];
ylist = [-1.5,-1.0,-1,-1.5,-1.5];
plot(xlist,ylist,'-r','lineWidth',2);

%% processing data

[x,y] = visualize_dynamics_manipulation(data,l,n);
