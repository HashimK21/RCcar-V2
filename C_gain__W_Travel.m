clear
clc

%pull values from csv

values = readtable('rear_data.csv'); %pull values from rear
%values = readtable('front_data.csv'); %pull values from front

m = values.Upper_Beam;
n = values.Lower_Beam;
l = values.Lower_arm_length;
u = values.Upper_arm_length;
hr = values.Ride;
sumh = values.Frame_h_ground;

deltay = 20.5; %for front
%deltay = 30; %for rear

od = 40; %for front
%od = 10; %for rear

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
Uy = sumh + ((u - m).*cosd(alpha)) + (y2 - sumh).*sind(alpha));

%damper base rotation
dx = n + ((l - n - od).*cosd(theta)) - ((y1 - hr).*sind(theta)); %x rotation
dy = hr + ((l - n - od).*sind(theta)) - ((y1 - hr).*cosd(theta)); %y rotation

dxf = (l - od) + (d.*(cosd(thetad)));
dyf = y1 + (d.*(sind(thetad)));

%rotaion limitations
distLim = ((Lx - Ux)^2) + ((Ly - Uy)^2);
dcomp = sqrt(((dxf - dx)^2) + ((dxf - df)^2);


