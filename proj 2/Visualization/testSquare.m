rectangle('Position',[0.5,0,2,1], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[-1.0,2,2,1], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[-1,-2,2,1], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[-3,-2.5,1,5], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[3.35,-1.25,0.5,3.75], 'LineWidth',2,'LineStyle','-');
hold on

N = length(mistake);
for ii = 2:2
    x = xS(mistake(ii)); y = yS(mistake(ii));
    theta = thetaS(mistake(ii)); L = sideLength(mistake(ii));
    [a,b,c,d] = get4Corners(x,y,theta,L);
    point1 = [x+0.5*L;y-0.5*L];
    point2 = [x+0.5*L;y+0.5*L];
    point3 = [x-0.5*L;y+0.5*L];
    point4 = [x-0.5*L;y-0.5*L];
    %xlist = [point1(1),point2(1),point3(1),point4(1),point1(1)];
    %ylist = [point1(2),point2(2),point3(2),point4(2),point1(2)];
    % plot(xlist,ylist,'-r','lineWidth',2);
    hold on;
    xlist = [a(1),b(1),c(1),d(1),a(1)];
    ylist = [a(2),b(2),c(2),d(2),a(2)];
    plot(xlist,ylist,'-r','lineWidth',2);
end

xlist = [-1.42175 -1.49865 2.20673 2.28363 -1.42175];
ylist = [3.73686 3.24281 2.66606 3.16011 3.73686];
plot(xlist,ylist,'-g','lineWidth',2);

xlist = [3.27447 3.27447 2.92447 2.92447 3.27447];
ylist = [3.11167 3.46167 3.46167 3.11167 3.11167];
plot(xlist,ylist,'-g','lineWidth',2);
