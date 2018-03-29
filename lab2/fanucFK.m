function [ T,fanuc_T ] = fanucFK( joint_angles,fanuc )

l_1 = fanuc.parameters.l_1;
l_2 = fanuc.parameters.l_2;
l_3 = fanuc.parameters.l_3;
l_4 = fanuc.parameters.l_4;
l_5 = fanuc.parameters.l_5;
l_6 = fanuc.parameters.l_6;

alpha = [0,pi/2,0,pi/2,-pi/2,pi/2];
a = [0,l_2,l_3,l_4,0,0];
d = [0,0,0,l_5,0,l_6];
theta = joint_angles;
theta(2) = theta(2) + pi/2;

fT = cell(1,6);
Ttmp = eye(4);
for ii = 1 : 6
    fT{ii} = dhtf(alpha(ii),a(ii),d(ii),theta(ii));
    Ttmp = Ttmp*fT{ii};
end

T = Ttmp;
fanuc_T = fT;
end
