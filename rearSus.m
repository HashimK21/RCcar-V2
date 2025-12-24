clc

tic();

tw = 220; %track width
wb = 352; %wheel base
tireD = 85; %tire diamater
tireW = 42; %tire width
y1 = 19; %ground to lower sus mount
xj = (tw/2) - (tireW/2); %position of upper and lower joints x, offset to allow bolt to be outside of wheel

set1 = 25:1:40; %range of values for m, upper chassis beam
set2 = 25:1:40; %range of values for n, lower chassis beam
set3 = 40:2:80; %range of values for hf
set4 = 15:2:20; %range of values for y2 offset from wheel top


[set1, set2, set3, set4] = ndgrid(set1, set2, set3, set4);
combinations = [set1(:), set2(:), set3(:), set4(:)];

m = set1(:);
n = set2(:);
hr = 20;
hf = set3(:);
sumh = hr + hf;
y2 = tireD - set4(:); %ground to upper sus mount

%IC location
xLnum = (hr.*(xj - n).*(xj - m)) - (n.*(y1 - hr).*(xj - m)) - (sumh.*(xj - n).*(xj - m)) + (m.*(y2 - sumh).*(xj - n));
xLdom =((y1 - hr).*(xj - m)) - ((y2 - sumh).*(xj - n));
xL = xLnum./xLdom;

yL = (-((y1 - hr).*(xL + n))./(xj - n)) + hr;
yLcheck = (-((y2 - sumh).*(xL + m))./(xj - m)) + sumh;

xRnum = (hr.*(xj - n).*(xj - m)) - (n.*(y1 - hr).*(xj - m)) - (sumh.*(xj - n).*(xj - m)) + (m.*(y2 - sumh).*(xj - n));
xRdom =((y2 - sumh).*(xj - n)) - ((y1 - hr).*(xj - m));
xR = xRnum./xRdom;

yR = (((y1 - hr).*(xR - n))./(xj - n)) + hr;
yRcheck = (((y2 - sumh).*(xR - m))./(xj - m)) + sumh;

%Roll centre location
xRCnum = (tw/2).*((yL .* (xR -(tw/2))) + (yR .* (xL + (tw/2))));
xRCdom = (yR .* (xL + (tw/2))) - (yL .* (xR - (tw/2)));
xRC = xRCnum ./ xRCdom;

yRC = (yL./(xL + (tw/2))).*(xRC + (tw/2));
yRCcheck = (yR./(xR - (tw/2))).*(xRC - (tw/2));

%arm lengths and angles
thetau = atand((y2 - sumh)./ (xj - m));
u = (xj - m)./cosd(thetau);

alpha = atand((hr - y1)./(xj - n));
l = (xj - n)./cosd(alpha);

%sorting
clearance = 7; %for material and design
batteryHeight = 35/2; %half of battery height
CGheight = hr + clearance + batteryHeight;
CGpCent_low = (CGheight*0.15);
CGpCent_high = (CGheight*0.2);

index_sort = (l > 0) & (abs(xL + xR) < 0.01) & (abs(xRC)<= 0)...
             & (abs(xRC) >= 0) & (yRC >= CGpCent_low) & (yRC <= CGpCent_high) ...
             & (xL >= (tw)) & (xL <= (tw*1.2)) & (l > u);

s_yRC = yRC(index_sort);
s_xRC = xRC(index_sort);
s_m = m(index_sort);
s_n = n(index_sort);
s_hr = (hr * ones(size(s_yRC)));
s_hf = hf(index_sort);
s_sumh = sumh(index_sort);
s_y2 = y2(index_sort);
s_xL = xL(index_sort);
s_yL = yL(index_sort);
s_xR = xR(index_sort);
s_yR = yR(index_sort);
s_thetau = thetau(index_sort);
s_u = u(index_sort);
s_alpha = alpha(index_sort);
s_l = l(index_sort);

s_y1 = (y1 * ones(size(s_y2)));


%print sorted data to csv
columnLabels = {'yRC', 'xRC', 'Upper_Beam', 'Lower_Beam',...
                'Ride', 'Frame_Height', 'Frame_h_ground', 'y1', 'y2',...
                'xL', 'yL', 'xR', 'yR', 'Upper_arm_angle',...
                'Upper_arm_length', 'Lower_arm_angle', 'Lower_arm_length'};

combinedData = [s_yRC(:), s_xRC(:), s_m(:), s_n(:), s_hr(:), s_hf(:), ...
                s_sumh(:), s_y1(:), s_y2(:), s_xL(:), s_yL(:), s_xR(:), s_yR(:),...
                s_thetau(:), s_u(:), s_alpha(:), s_l(:)];
fileID = fopen('rear_data.csv', 'w');
fprintf(fileID, '%s,', columnLabels{1:end-1});
fprintf(fileID, '%s\n', columnLabels{end});
csvwrite('rear_data.csv', combinedData, '-append', 'delimiter', ',');
fclose(fileID);

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])

