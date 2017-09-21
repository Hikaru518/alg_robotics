N = length(mistake);
for ii = 1:N
    plot(x(mistake),y(mistake),'.');
end

rectangle('Position',[0.5,0,2,1], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[-1.0,2,2,1], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[-1,-2,2,1], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[-3,-2.5,1,5], 'LineWidth',2,'LineStyle','-');
rectangle('Position',[3.35,-1.25,0.5,3.75], 'LineWidth',2,'LineStyle','-');


