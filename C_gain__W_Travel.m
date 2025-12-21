clear
clc

tic();

%pull values from csv
values_data = dlmread('rear_data.csv', ',', 1, 0);
%values_data = dlmread('front_data.csv', ',', 1, 0); %pull values from front
values = values_data(1,:);

m = values(3);
n = values(4);
l = values(17);
u = values(15);
hr = values(5);
sumh = values(7);
y1 = values(8);
y2 = values(9);

deltay = y2 - y1;

%od = 40; %for front
od = 10; %for rear

d = 110; %damper length
dcompFull = 90; %length of fully compressed damper

thetad = 110; %angle of damper anticlockwise from x axis

%set theta and alpha (angles for arm rotation)
set1 = 0:1:15;
set2 = 0:0.25:15;

[set1, set2] = ndgrid(set1, set2);
combinations = [set1(:), set2(:)];

theta = set1(:);
alpha = set2(:);

%lower arm rotaion
Lx = n + ((l - n).*cosd(theta)) - ((y1 - hr).*sind(theta));
Ly = hr + ((l - n).*sind(theta)) + ((y1 - hr).*cosd(theta));

%upper arm rotation
Ux = m + ((u - m).*cosd(alpha)) - ((y2 - sumh).*sind(alpha));
Uy = sumh + (((u - m).*sind(alpha)) + (y2 - sumh).*cosd(alpha));

%damper base rotation
dx = n + ((l - n - od).*cosd(theta)) - ((y1 - hr).*sind(theta)); %x rotation
dy = hr + ((l - n - od).*sind(theta)) + ((y1 - hr).*cosd(theta)); %y rotation

dxf = (l - od) + (d.*(cosd(thetad)));
dyf = y1 + (d.*(sind(thetad)));

%rotaion limitations
distLim = ((Lx - Ux).^2) + ((Ly - Uy).^2);
dcomp = sqrt(((dxf - dx).^2) + ((dyf - dy).^2));

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])