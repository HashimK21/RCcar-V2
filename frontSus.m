clc

tic();

set1 = 30:2:45; %range of values for m, upper chassis beam
set2 = 30:2:45; %range of values for n, lower chassis beam
set3 = 30:1:60; %range of values for hf
set4 = 18:2:40; %range of values for deltay

[set1, set2, set3, set4] = ndgrid(set1, set2, set3, set4);
combinations = [set1(:), set2(:), set3(:), set4(:)];

m = set1(:);
n = set2(:);
hr = 20;
hf = set3(:);
sumh = hr + hf;

%KPI offsets
thetak = 9.5;
KPL = -8.6;
tireW = 34; %tire width
tw = 220; %track width
wb = 352; %wheel base
tireD = 85; %tire diamater
xj = (tw/2) - (tireW/2);
y1 = 19;
deltay = set4(:);
y2 = y1 + deltay;
c = deltay .* tand(thetak);
KPU = KPL + c;
u = xj - KPU;
l = xj - KPL;
x = ((-y2 .* (u - l)) ./ deltay) + u;
rs = (tw/2) - x;
k = (KPU + KPL)./ 2;
alphac = 7;
zc = deltay .* tand(alphac);


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
clearance = 7; %for material and design
batteryHeight = 35/2; %half of battery height
CGheight = hr + clearance + batteryHeight;
CGpCent_low = (CGheight*0.17);
CGpCent_high = (CGheight*0.4);

index_sort = (l > 0) & (abs(xL + xR) < 0.01) & (abs(xRC)<= 0)...
             & (abs(xRC) >= 0) & (yRC >= CGpCent_low) & (yRC <= CGpCent_high) ...
             & (xL >= (tw*0.85)) & (xL <= (tw*2)) & (l > u);

s_yRC = yRC(index_sort);
s_xRC = xRC(index_sort);
s_m = m(index_sort);
s_n = n(index_sort);
s_hr = (hr * ones(size(s_yRC)));
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
s_y1 = (y1 * ones(size(s_yRC)));
s_y2 = y2(index_sort);
%s_y2 = (y2 * ones(size(s_yRC)));
s_KPU = KPU(index_sort);
%s_KPU = (KPU * ones(size(s_yRC)));
s_KPL = (KPL * ones(size(s_yRC)));
s_rs = rs(index_sort);
%s_rs = (rs * ones(size(s_yRC)));
s_k = k(index_sort);
%s_k = (k * ones(size(s_yRC)));
s_zc = zc(index_sort);
%s_zc = (zc * ones(size(s_yRC)));
s_alphac = (alphac * ones(size(s_yRC)));

%print sorted data to csv
columnLabels = {'yRC', 'xRC', 'Upper_Beam', 'Lower_Beam',...
                'Ride', 'Frame_Height', 'Frame_h_ground', 'y1', 'y2',...
                'xL', 'yL', 'xR', 'yR', 'Upper_arm_angle',...
                'Upper_arm_length', 'Lower_arm_angle', 'Lower_arm_length', 'KPU', 'KPL', 'Scrub Radius', ...
                'King Pin offset', 'Caster offset', 'Caster angle',};

combinedData = [s_yRC(:), s_xRC(:), s_m(:), s_n(:), s_hr(:), s_hf(:), ...
                s_sumh(:), s_y1(:), s_y2(:), s_xL(:), s_yL(:), s_xR(:), s_yR(:),...
                s_thetau(:), s_u(:), s_alpha(:), s_l(:), s_KPU(:), s_KPL(:), s_rs(:), ...
                s_k(:), s_zc(:), s_alphac(:)];

fileID = fopen('front_data.csv', 'w');
fprintf(fileID, '%s,', columnLabels{1:end-1});
fprintf(fileID, '%s\n', columnLabels{end});
csvwrite('front_data.csv', combinedData, '-append', 'delimiter', ',');
fclose(fileID);

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])


