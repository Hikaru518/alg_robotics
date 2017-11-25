function [x,y] = visualize_dynamics_manipulation(data,l,n)
[a b] = size(data);

x = zeros(a,n+1);
y = zeros(size(x));
phi = [];
phi = [phi data(:,1)];
for i = 2:n
   phi = [phi,phi(:,i-1)+data(:,i)];
end

x(:,n+1) = 0*cos(phi(:,n));
y(:,n+1) = 0*sin(phi(:,n));
for i = n:-1:1
    x(:,i) = x(:,i+1) + l(i)*cos(phi(:,i));
    y(:,i) = y(:,i+1) + l(i)*sin(phi(:,i));
end

for i = 1:a
    for j = 1:n
        line([x(i,j),x(i,j+1)],[y(i,j),y(i,j+1)])
        hold on;
    end
end
x
y

x1 = x(:,1);
y1 = y(:,1);
for i = 1:(length(x1)-1)
    line([x1(i),x1(i+1)],[y1(i),y1(i+1)],'Color','Green');
end
