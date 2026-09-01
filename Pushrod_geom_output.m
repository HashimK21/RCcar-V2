clear 

tic();

od = 21;
set1 = 40:5:65; 
set2 = 91:1:100;
y1 = 20.5;
n = 33;
l = 68.64553882081486 * cos(2.087114971053753);

[set1, set2] = ndgrid(set1, set2);
combinations = [set1(:), set2(:)];

d = set1(:);
theta = set2(:);

xpu = -(n + l - od) + (d.*cosd(theta));
ypu = y1 + (d.*sind(theta));

columnLabels = {'d', 'theta', 'xpu', 'ypu'};

fid = fopen('pushrod_data.csv', 'w');
fprintf(fid, '%s,%s,%s,%s\n', columnLabels{:});
fprintf(fid, '%d,%d,%.15f,%.15f\n', [d(:), theta(:), xpu(:), ypu(:)].');
fclose(fid);

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])