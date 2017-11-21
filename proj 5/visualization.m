%% Nov. 21

clear all
clc

q1 = rand(1000,1);
q2 = rand(1000,1);
q3 = rand(1000,1);

l1 = 0.5;
l2 = 0.5;
l3 = 0.5;

[x,y] = visualize_dynamics_manipulation(q1,q2,q3,l1,l2,l3);
