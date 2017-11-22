function [x,y] = visualize_dynamics_manipulation(q1,q2,q3,l1,l2,l3)
x = zeros(length(q1),4);
y = zeros(size(x));
phi1 = q1;
phi2 = q1+q2;
phi3 = q1+q2+q3;
x(:,3) = l3*cos(phi3); % x3 
y(:,3) = l3*sin(phi3); % y3
x(:,2) = x(:,3)+l2*cos(phi2); % x2
y(:,2) = y(:,3)+l2*sin(phi2); % y2
x(:,1) = x(:,2)+l1*cos(phi1); % x1
y(:,1) = y(:,2)+l1*sin(phi1); % y1
for i = 1:length(q1)
    line([x(i,1),x(i,2)],[y(i,1),y(i,2)])
    line([x(i,2),x(i,3)],[y(i,2),y(i,3)])
    line([x(i,3),x(i,4)],[y(i,3),y(i,4)])
end
end

