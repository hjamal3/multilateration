n = 10;
r = [];
theta = [];

xymin = 0;
xymax = 100;
x=xymin+randn(1,n)*(xymax-xymin);
y=xymin+randn(1,n)*(xymax-xymin);

robotx = randn()*30;
roboty = randn()*30;

close all
scatter(x,y,'b');
hold on;
scatter(robotx,roboty,'r');

sigma = 0.2;

d = sqrt((x - robotx).*(x - robotx) + (y - roboty).*(y - roboty)) + randn(1,n)*sigma;

fprintf('%8.2f, %8.3f,%8.3f,\n', [x(1:end-1),y(1:end-1),d(1:end-1)]')
fprintf('%8.2f, %8.3f,%8.3f\n', [x(end),y(end),d(end)]')


