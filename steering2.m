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
set1 = 19:1:25; %values for h, radius of rotation
set2 = 10:1:15; %values for steering arm, s
set3 = 100:2:110; %values for tie rod, t
set4 = 50:2:60; %values for r/2
set5 = 20:2:30; %values for zr -- rack position in z
set6 = 45:1:53; %values for x offset from pivot to tie rod joint (cx)
set7 = 10:1:20; %values for z offset from pivot top end of steering arm

[set1, set2, set3, set4, set5, set6, set7] = ndgrid(set1, set2, set3, set4, set5 , set6, set7);
combinations = [set1(:), set2(:), set3(:), set4(:), set5(:), set6(:), set7(:)];

%steering angle in radians
theta_vals = linspace(0, 0.4363, 25);


num_cases = numel(set1);
num_theta = numel(theta_vals);
num_val = 7; %number of ngrid variables

% Preallocate output matrix
output_pos = zeros(num_cases, num_val + num_theta);
output_neg = zeros(num_cases, num_val + num_theta);

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

    end

        % One row per case: 5 constants + values for angle dependants
        output_pos(i, :) = [h, s, t, zr, rO2, cx, cz, thetaWheel_vals_pos];
        output_neg(i, :) = [h, s, t, zr, rO2, cx, cz, thetaWheel_vals_neg];

end

disp('finished calculations')

% -- Purge bad rows --
%location of thetaS = 0
[~, idxZero] = min(abs(theta_vals - 0));
thetaIndex = num_val + idxZero;
thetaIndexLast = num_val + num_theta;

%First Pass: Filter based on thetaIndex and thetaIndexLast conditions
cond_pos_pass1 = (abs(output_pos(:, thetaIndex)) < 1) & ...
                 (abs(output_pos(:, thetaIndex)) > 0) & ...
                 (output_pos(:, thetaIndexLast) < 0);
intermediate_pos = output_pos(cond_pos_pass1, :);

cond_neg_pass1 = (abs(output_neg(:, thetaIndex)) < 1) & ...
                 (abs(output_neg(:, thetaIndex)) > 0) & ...
                 (output_neg(:, thetaIndexLast) < 0);
intermediate_neg = output_neg(cond_neg_pass1, :);

disp(['Pass 1 complete. Pos candidates: ', num2str(size(intermediate_pos, 1)), ', Neg candidates: ', num2str(size(intermediate_neg, 1))]);


%Second Pass: Filter based on incrementation conditions


max_jump = 2.8; % Max allowable jump between thetaWheel values in degrees
min_theta_inc = 0.7; % Min allowable incrament of the absolute value of thetaWheel values in degrees

% For positive branch
[Rows_pos, Cols_pos] = size(intermediate_pos);
tempMatrix = zeros(Rows_pos, Cols_pos);
validRowCount = 0;
for rowIndex = 1:Rows_pos
    currentRow_pos = intermediate_pos(rowIndex, :);
    thetaWheel_pos_values = currentRow_pos(num_val+1:end);
    diff_pos = diff(thetaWheel_pos_values);
    diff_abs_pos = diff(abs(thetaWheel_pos_values));

    % Increment checks
    cond_max_jump = all(abs(diff_pos) < max_jump);
    cond_mag_not_decreasing = all(diff_abs_pos >= 0);
    cond_min_inc = all(diff_abs_pos(diff_abs_pos > 0) > min_theta_inc);

    if cond_max_jump && cond_mag_not_decreasing && cond_min_inc
        validRowCount = validRowCount + 1;
        tempMatrix(validRowCount, :) = currentRow_pos;
    end
end
output_final_pos = tempMatrix(1:validRowCount, :);


% For negative branch
[Rows_neg, Cols_neg] = size(intermediate_neg);
tempMatrix2 = zeros(Rows_neg, Cols_neg);
validRowCount2 = 0;
for rowIndex = 1:Rows_neg
    currentRow_neg = intermediate_neg(rowIndex, :);
    thetaWheel_neg_values = currentRow_neg(num_val+1:end);
    diff_neg = diff(thetaWheel_neg_values);
    diff_abs_neg = diff(abs(thetaWheel_neg_values));

    % Increment checks
    cond_max_jump = all(abs(diff_neg) < max_jump);
    cond_mag_not_decreasing = all(diff_abs_neg >= 0);
    cond_min_inc = all(diff_abs_neg(diff_abs_neg > 0) > min_theta_inc);

    if cond_max_jump && cond_mag_not_decreasing && cond_min_inc
        validRowCount2 = validRowCount2 + 1;
        tempMatrix2(validRowCount2, :) = currentRow_neg;
    end
end
output_final_neg = tempMatrix2(1:validRowCount2, :);

disp(['Pass 2 complete. Final pos cases: ', num2str(size(output_final_pos, 1)), ', Final neg cases: ', num2str(size(output_final_neg, 1))]);

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

disp('done')
