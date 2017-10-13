function h = circleLine3(x,y,z,r,theta)
hold on
th = 0:pi/30:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
zunit = ones(length(th))*z;

h = plot3(xunit, yunit, zunit,'b');
xline = [x;r * cos(theta) + x];
yline = [y;r * sin(theta) + y];
zline = [z;z];
h = plot3(xline,yline,zline,'g');

hold off
end