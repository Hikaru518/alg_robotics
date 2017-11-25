%% Nov. 21
% 
% clear all
% clc
% 
% q1 = rand(1000,1);
% q2 = rand(1000,1);
q3 = zeros(19,1);

l1 = 1;
l2 = 1;
l3 = 0;

%%
xlist = [-0.5,-0.5,0,0,-0.5];
ylist = [-1.5,-1.0,-1.0,-1.5,-1.5];
plot(xlist,ylist,'-r','lineWidth',2);


%%
[x,y] = visualize_dynamics_manipulation(q1,q2,q3,l1,l2,l3);

