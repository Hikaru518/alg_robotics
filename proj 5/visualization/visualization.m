% 
% clear all
% clc
%
%% setting parameters
n = 3;
l = [0.3,0.6,0.9];
data = importdata('9.txt');
envType = 1; % 0 is freespace, 1 is obstacle

%% setting obstacles
if (envType == 1)
    xlist = [-0.5,-0.5,0,0,-0.5];
    ylist = [-1.5,-1.0,-1.0,-1.5,-1.5];
    plot(xlist,ylist,'-r','lineWidth',2);
end

%% processing data

[x,y] = visualize_dynamics_manipulation(data,l,n);