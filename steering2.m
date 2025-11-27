clear
clc
pkg load parallel

%global constants
tw = 220; %track width
wb = 352; %wheelbase
tireD = 85; %tire diamater
tireW = 34; %tire width
k = -6.93; %offset for KPI mid point projection

xj = (tw/2) - (tireW/2); %tire wall, used to place sus joints

%Rack Position
set1 = 30:10:70; %values for h, radius of rotation
set2 = 20:5:50; %values for steering arm, s
set3 = 50:10:150; %values for tie rod, t
set4 = 30:5:60; %values for r/2
set5 = 30:10:100; %values for zr -- rack position in z
set6 = 10:10:50; %values for x offset from pivot to tie rod joint (cx)
set7 = 20:2.5:30; %values for z offset from pivot top end of steering arm

[set1, set2, set3, set4, set5, set6, set7] = ndgrid(set1, set2, set3, set4, set5 , set6, set7);
combinations = [set1(:), set2(:), set3(:), set4(:), set5(:), set6(:), set7(:)];

%steering angle in radians
theta_vals = linspace(0, (pi/6), 30);

num_cases = numel(set1);
num_theta = numel(theta_vals);
num_val = 7; %number of ngrid variables

% Preallocate output matrix
output_pos = zeros(num_cases, num_val + num_theta);
output_neg = zeros(num_cases, num_val + num_theta);
test = zeros(num_cases, num_val + num_theta);

disp('created ngrid and output matrix')

for i = 1:numel(set1)
    h = set1(i);
    s = set2(i);
    t = set3(i);
    rO2 = set4(i);
    zr = set5(i);
    cxComp = set6(i);
    czComp = set7(i);

    thetaWheel_vals_pos = zeros(1, num_theta);
    thetaWheel_vals_neg = zeros(1, num_theta);
    test_vals =  zeros(1, num_theta);
    parfor p = 1:numel(theta_vals)
        thetaS = theta_vals(p);
        %thetaS = 0;
        %finding rack displacement
        Delx = h .* sin(thetaS);

        xr = Delx + rO2;

        %Wheel angle
        xp = xj - k; %x value for wheel pivot
        zp = 0;

        cx = -(cxComp); %distance to wheel pivot from steering arm tie rod joint, x
        cz = -(czComp + s); %distance to wheel pivot from steering arm tie rod joint, z

        dx = xp - xr;
        dz = zp - zr;

        Tnum = (t^2) - ((cx)^2) - ((cz)^2) - ((dx)^2) - ((dz)^2);
        T = Tnum ./ 2; %constant terms of each case

        alpha = (dx .* cx) + (dz .* cz);
        beta = (dz .* cx) - (dx .* cz);

        R = sqrt(((alpha)^2) + ((beta)^2));

        % safe evaluation of acos argument and correct atan quadrant
        if R == 0
            thetaWheelRAD = NaN;
        else
            arg = T ./ R;
            arg = min(max(arg, -1), 1);        % clamp to [-1,1]
            phi = atan2(beta, alpha);          % correct quadrant
            thetaWheelFromHorz1 = phi + acos(arg);
            thetaWheelFromHorz2 = phi - acos(arg);
            thetaWheelRAD1 = (pi/2) - thetaWheelFromHorz1;
            thetaWheelRAD2 = (pi/2) - thetaWheelFromHorz2;
        end

        thetaWheel_pos = thetaWheelRAD1 .* (180/pi);
        thetaWheel_neg = thetaWheelRAD2 .* (180/pi);

        thetaWheel_vals_pos(p) = thetaWheel_pos;   % store output for positive acos
        thetaWheel_vals_neg(p) = thetaWheel_neg;   % store output for negative acos
        %test_vals(p) = thetaWheelRAD;

    end

        % One row per case: 5 constants + values for angle dependants
        output_pos(i, :) = [h, s, t, zr, rO2, cx, cz, thetaWheel_vals_pos];
        output_neg(i, :) = [h, s, t, zr, rO2, cx, cz, thetaWheel_vals_neg];
        %test(i, :) = [h, s, t, zr, rO2, test_vals];

end

disp('finished calculations')

% -- Purge bad rows --
[Rows_pos, Cols_pos] = size(output_pos);
[Rows_neg, Cols_neg] = size(output_neg);
tempMatrix = zeros(Rows_pos, Cols_pos);
tempMatrix2 = zeros(Rows_neg, Cols_neg);
validRowCount = 0;
validRowCount2 = 0;
% Track valid rows
validIndices = false(Rows_pos, 1);
validIndices2 = false(Rows_neg, 1);

% location of thetaS = 0
[~, idxZero] = min(abs(theta_vals - 0));
% the correct column in outputs
thetaIndex = num_val + idxZero;
thetaIndexLast = num_val + num_theta;

% Merge pos/neg purging into a single loop to keep logic centralized.
maxRows = max(Rows_pos, Rows_neg);
max_jump = 6; % Max allowable jump between thetaWheel values in degrees
for rowIndex = 1:maxRows
    % Handle positive-acos outputs when available
    if rowIndex <= Rows_pos
        currentRow_pos = output_pos(rowIndex, :);

        % Check for large jumps in thetaWheel values
        thetaWheel_pos_values = currentRow_pos(num_val+1:end);
        diff_pos = diff(thetaWheel_pos_values);
        
        % conditions for acceptance (positive branch)
        cond_theta0_under_1_pos = abs(currentRow_pos(thetaIndex)) < 1;
        cond_theta0_over_0_pos = abs(currentRow_pos(thetaIndex)) > 0;
        cond_thetaLast_neg_pos = currentRow_pos(thetaIndexLast) < 0;
        cond_no_large_jumps_pos = all(abs(diff_pos) < max_jump);

        if cond_theta0_under_1_pos && cond_theta0_over_0_pos && cond_thetaLast_neg_pos && cond_no_large_jumps_pos
            validRowCount = validRowCount + 1;
            tempMatrix(validRowCount, :) = currentRow_pos;
            validIndices(rowIndex) = true; % Mark as valid (pos index)
        end
    end

    % Handle negative-acos outputs when available
    if rowIndex <= Rows_neg
        currentRow_neg = output_neg(rowIndex, :);

        % Check for large jumps in thetaWheel values
        thetaWheel_neg_values = currentRow_neg(num_val+1:end);
        diff_neg = diff(thetaWheel_neg_values);
        

        % conditions for acceptance (negative branch)
        cond_theta0_under_1_neg = abs(currentRow_neg(thetaIndex)) < 1;
        cond_theta0_over_0_neg = abs(currentRow_neg(thetaIndex)) > 0;
        cond_thetaLast_neg_neg = currentRow_neg(thetaIndexLast) < 0;
        cond_no_large_jumps_neg = all(abs(diff_neg) < max_jump);

        if cond_theta0_under_1_neg && cond_theta0_over_0_neg && cond_thetaLast_neg_neg && cond_no_large_jumps_neg
            validRowCount2 = validRowCount2 + 1;
            tempMatrix2(validRowCount2, :) = currentRow_neg;
            validIndices2(rowIndex) = true; % Mark as valid (neg index)
        end
    end
end

output_final_pos = tempMatrix(1:validRowCount, :);
output_final_neg = tempMatrix2(1:validRowCount2, :);
test_final = test(validIndices, :); % Filter test matrix using validIndices

disp('Completed Purge')

% -- Build header row --
thetaWheel_headers = cell(1, num_theta);
    for g = 1:num_theta
        thetaWheel_headers{g} = ['thetaWheel_' num2str(g)];
    end
headers = ["h", "s", "t", "zr", "r/2", "cx", "cz", thetaWheel_headers];

disp('Created Headers')

fid = fopen('steering_pos.csv', 'w');
header_line = strjoin(headers, ',');
fprintf(fid, '%s\n', header_line);

% -- write matrix --
[rows, cols] = size(output_final_pos);
if rows > 0
    fmt = [repmat('%g,', 1, cols-1) '%g\n'];
    for r = 1:rows
        fprintf(fid, fmt, output_final_pos(r, :));
    end
end
fclose(fid);

fid2 = fopen('steering_neg.csv', 'w');
header_line = strjoin(headers, ',');
fprintf(fid2, '%s\n', header_line);

[rows, cols] = size(output_final_neg);
if rows > 0
    fmt = [repmat('%g,', 1, cols-1) '%g\n'];
    for r = 1:rows
        fprintf(fid2, fmt, output_final_neg(r, :));
    end
end
fclose(fid2);

disp('Print to CSV')

% % -- test cases --
% % -- Build header row for tests --
% test_headers = cell(1, num_theta);
%     for g_test = 1:num_theta
%         test_headers{g_test} = ['testedVariable_' num2str(g_test)];
%     end
% headers_test = ["h", "s", "t", "zr", "r/2", test_headers];

% disp('Created Test Headers')

% fid = fopen('test_vals.csv', 'w');
% header_line_test = strjoin(headers_test, ',');
% fprintf(fid, '%s\n', header_line_test);

% % -- write test matrix --
% [rowsT, colsT] = size(test_final);
% if rowsT > 0
%     fmtT = [repmat('%g,', 1, colsT-1) '%g\n'];
%     for r = 1:rowsT
%         fprintf(fid, fmtT, test_final(r, :));
%     end
% end
% fclose(fid);

% disp('Print Test to CSV')

disp('done')
