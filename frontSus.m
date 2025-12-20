clc

tic();

tw = 220; %track width
wb = 352; %wheel base
tireD = 85; %tire diamater
tireW = 34; %tire width
thetaK = 9.5; %KPI angle
y1 = 20.5; %ground to lower sus mount
y2 = 40.5; %ground to upper sus mount
u = 98.25; %upper sus mount in x
l = 101.6; %lower sus mount in x

set1 = 11.5:2:30; %range of values for m, upper chassis beam
set2 = 11.5:2:30; %range of values for n, lower chassis beam
set3 = 7:1:30; %range of values for hr
set4 = 7:1:20; %range of values for hf

[set1, set2, set3, set4] = ndgrid(set1, set2, set3, set4);
combinations = [set1(:), set2(:), set3(:), set4(:)];

m = set1(:);
n = set2(:);
hr = set3(:);
hf = set4(:);
sumh = hr + hf;

%IC locataion
xLnum = (sumh .* (l - n) .* (u - m)) - (m .* (y2 - sumh) .* (l - n)) - (hr .* (u - m) .* (l - n)) + (n .* (y1 - hr) .* (u - m));
xLdom = ((y2 - sumh) .* (l - n)) - ((y1 - hr) .* (u - m));
xL = xLnum ./ xLdom;

yL = ((-(y2 - sumh) .* (xL + m)) ./ (u - m)) + sumh;
yLcheck = ((-(y1 - hr) .* (xL + n)) ./ (l - n)) + hr;

xRnum = (sumh .* (l - n) .* (u - m)) - (m .* (y2 - sumh) .* (l - n)) - (hr .* (u - m) .* (l - n)) + (n .* (y1 - hr) .* (u - m));
xRdom = ((y1 - hr) .* (u - m)) - ((y2 - sumh) .* (l - n));
xR = xRnum ./ xRdom;

yR = (((y2 - sumh) ./ (u - m)) .* (xR - m)) + sumh;
yRcheck = (((y1 - hr) ./ (l - n)) .* (xR - n)) + hr;

%Roll Centre location
xRCnum = (tw/2) .* ((yL .* (xR - (tw/2))) + (yR .* (xL + (tw/2))));
xRCdom = (yR .* (xL + (tw/2))) - (yL .* (xR - (tw/2)));
xRC = xRCnum ./ xRCdom;

yRC = (yR ./ (xR - (tw/2))) .* (xRC - (tw/2));
yRCcheck = (yL ./ (xL + (tw/2))) .* (xRC + (tw/2));

%arm length and angles
thetau = atand((y2 - sumh)./ (u - m));
up = (u - m)./cosd(thetau);

alpha = atand((hr - y1)./(l - n));
low = (l - n)./cosd(alpha);

%sorting
clearance = 5; %for material and design
batteryHeight = 35/2; %half of battery height
CGheight = hr + clearance + batteryHeight;
CGpCent = (CGheight/100)*30;

index_sort = (l > 0) & (abs(xL + xR) < 0.01) & (abs(xRC)<= 0)...
             & (abs(xRC) >= 0) & (yRC >= CGpCent);

s_yRC = yRC(index_sort);
s_xRC = xRC(index_sort);
s_m = m(index_sort);
s_n = n(index_sort);
s_hr = hr(index_sort);
s_hf = hf(index_sort);
s_sumh = sumh(index_sort);
s_xL = xL(index_sort);
s_yL = yL(index_sort);
s_xR = xR(index_sort);
s_yR = yR(index_sort);
s_thetau = thetau(index_sort);
s_u = up(index_sort);
s_alpha = alpha(index_sort);
s_l = low(index_sort);

%print sorted data to csv
columnLabels = {'yRC', 'xRC', 'Upper Beam', 'Lower Beam',...
                'Ride', 'Frame Height', 'Frame h from ground',...
                'xL', 'yL', 'xR', 'yR', 'Upper arm angle',...
                'Upper arm length', 'Lower arm angle', 'Lower arm length'};
combinedData = [s_yRC(:), s_xRC(:), s_m(:), s_n(:), s_hr(:), s_hf(:), ...
                s_sumh(:), s_xL(:), s_yL(:), s_xR(:), s_yR(:),...
                s_thetau(:), s_u(:), s_alpha(:), s_l(:)];
fileID = fopen('front_data.csv', 'w');
fprintf(fileID, '%s,', columnLabels{1:end-1});
fprintf(fileID, '%s\n', columnLabels{end});
csvwrite('front_data.csv', combinedData, '-append', 'delimiter', ',');
fclose(fileID);

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])


