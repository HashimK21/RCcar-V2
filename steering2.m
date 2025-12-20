clc

%global constants
tw = 220; %track width
wb = 352; %wheelbase
tireD = 85; %tire diamater
tireW = 34; %tire width
k = -6.93; %offset for KPI mid point projection
rs = 4.97; %scrub radius

%-- Create ndgrid of variables --
set1 = 20:1:40; %values for rack offset from front axel, zr
set2 = 60:1:80; %values for r/2
set3 = 8:1:12; %steering arm offset from pivot
set4 = 3:1:5; %differing of size


[set1, set2, set3, set4] = ndgrid(set1, set2, set3, set4);
combinations = [set1(:), set2(:), set3(:), set4(:)];

%steering angle in degrees
ServoAngle = 25; 

StepSize = (2*ServoAngle) + 1;
theta_vals = linspace(-(ServoAngle), ServoAngle, StepSize);


num_cases = numel(set1);
num_theta = numel(theta_vals);
num_val = 6; %number of ndgrid variables + constants that are output before angle dependant values

% Preallocate output matrix
output_pos = zeros(num_cases, num_val + num_theta);
output_neg = zeros(num_cases, num_val + num_theta);

disp('created ndgrid and output matrix')

h = 30; 
xj = (tw/2) - (tireW/2); 
xp = xj - k; 
zp = 0;

tic();
for i = 1:numel(set1)
    zr = -set1(i);
    rO2 = set2(i);

    cxComp = set3(i); %steering lever x component
    czComp = cxComp + set4(i); %steering lever z component

    cx = -(cxComp); %distance to wheel pivot from steering arm tie rod joint, x
    cz = -(czComp); %distance to wheel pivot from steering arm tie rod joint, z

    %calculating tie rod length, t
    tx = (xp + cx - rO2)^2;
    tz = (zp + cz - zr)^2;
    t = (sqrt(tx + tz)) .* 1.02;

    thetaWheel_vals_pos = zeros(1, num_theta);
    thetaWheel_vals_neg = zeros(1, num_theta);
    test_arg = zeros(1, num_theta);
    % test_R = zeros(1, num_theta);
    % test_T = zeros(1, num_theta);
    % test_phi = zeros(1, num_theta);
    for p = 1:numel(theta_vals)
        thetaS = theta_vals(p);
        
        %finding rack displacement
        Delx = h .* sind(thetaS);
        xr = Delx + rO2;

        %Wheel angle
        dx = xp - xr;
        dz = zp - zr; 

        Tnum = (t^2) - ((cx)^2) - ((cz)^2) - ((dx)^2) - ((dz)^2);
        T = Tnum ./ 2; %constant terms of each case

        alpha = (dx .* cx) + (dz .* cz);
        beta = (dz .* cx) - (dx .* cz);

        R = sqrt(((alpha)^2) + ((beta)^2));

        % safe evaluation of acos argument and correct atan quadrant
        if R == 0
            thetaWheelRAD_pos = NaN;
            thetaWheelRAD_neg = NaN;
        else
            arg = T ./ R;
            arg = min(max(arg, -1), 1); % clamp to [-1,1]
            phi = atan2(beta, alpha); % correct quadrant
    
            % Raw angles
            thetaWheelRAD_pos_raw = phi + acos(arg);
            thetaWheelRAD_neg_raw = phi - acos(arg);
    
            % UNWRAP both to [-180,180] principal range
            thetaWheelRAD_pos = mod(thetaWheelRAD_pos_raw + pi, 2*pi) - pi;
            thetaWheelRAD_neg = mod(thetaWheelRAD_neg_raw + pi, 2*pi) - pi;
        end

        thetaWheel_pos = rad2deg(thetaWheelRAD_pos);
        thetaWheel_neg = rad2deg(thetaWheelRAD_neg);

        thetaWheel_vals_pos(p) = thetaWheel_pos;
        thetaWheel_vals_neg(p) = thetaWheel_neg;
        test_arg(p) = arg;
        test_R(p) = R;
        test_T(p) = T;
        test_phi(p) = phi;

    end

        % One row per case: 5 constants + values for angle dependants
        output_pos(i, :) = [h, t, rO2, zr, cx, cz, thetaWheel_vals_pos];
        output_neg(i, :) = [h, t, rO2, zr, cx, cz, thetaWheel_vals_neg];
        output_arg(i, :) = [h, t, rO2, zr, cx, cz, test_arg];
        % output_R(i, :) = [h, t, rO2, zr, cx, cz, test_R];
        % output_T(i, :) = [h, t, rO2, zr, cx, cz, test_T];
        % output_phi(i, :) = [h, t, rO2, zr, cx, cz, test_phi];

end

end_time = toc();
disp(['finished calculations in time:' num2str(end_time) ' seconds']);

% -- Purge bad rows --
%location of thetaS = 0
[~, idxZero] = min(abs(theta_vals));
thetaIndex = num_val + idxZero;
thetaIndexLast = num_val + num_theta;
thetaIndexFirst = num_val + 1;
midpoint = ceil(num_theta / 2);

%First pass variables
max_upper_lim = 0.1;
min_lock = 0;
%Second pass variables
max_jump = 10000;
tollerance = 10000;
%Third pass variables
ackLimLow = 0; %min ackermann percentage
ackLimHigh = 100; %max ackermann percentage
TurningCircleLim = 300; %min turning circle diameter in mm


% -- First Pass: Filter based on thetaIndex and thetaIndexLast conditions --
cond_pos_pass1 = (abs(output_pos(:, thetaIndex)) <= max_upper_lim) ...
                 & (abs(output_pos(:, thetaIndex)) >= 0) ...
                 & (output_pos(:, thetaIndexLast) < 0) ...
                 & (output_pos(:, thetaIndexFirst) > 0) ...
                 & (abs(output_pos(:, thetaIndexLast)) <= abs(output_pos(:, thetaIndexFirst))) ...
                 & (abs(output_pos(:, thetaIndexFirst)) <= 45) ...
                 & (abs(output_pos(:, thetaIndexLast)) >= min_lock) ...
                 & (abs(output_pos(:, thetaIndexFirst)) >= min_lock); 
intermediate_pos = output_pos(cond_pos_pass1, :);

cond_neg_pass1 = (abs(output_neg(:, thetaIndex)) <= max_upper_lim) ...
                 & (abs(output_neg(:, thetaIndex)) >= 0) ...
                 & (output_neg(:, thetaIndexLast) < 0) ...
                 & (output_neg(:, thetaIndexFirst) > 0) ...
                 & (abs(output_neg(:, thetaIndexLast)) <= abs(output_neg(:, thetaIndexFirst))) ...
                 & (abs(output_neg(:, thetaIndexFirst)) <= 45) ...
                 & (abs(output_neg(:, thetaIndexLast)) >= min_lock) ...
                 & (abs(output_neg(:, thetaIndexFirst)) >= min_lock);
intermediate_neg = output_neg(cond_neg_pass1, :);

disp(['Pass 1 complete. Pos cases: ', num2str(size(intermediate_pos, 1)), ', Neg cases: ', num2str(size(intermediate_neg, 1))]);


% -- Second Pass: Filter based on incrementation conditions --
% For positive branch
[Rows_pos, Cols_pos] = size(intermediate_pos);
tempMatrix = zeros(Rows_pos, Cols_pos);
validRowCount = 0;
for rowIndex = 1:Rows_pos
    currentRow_pos = intermediate_pos(rowIndex, :);
    thetaWheel_pos_values = currentRow_pos(num_val+1:end);
    diff_pos = diff(thetaWheel_pos_values);
    diff_abs_pos = diff(abs(thetaWheel_pos_values));

    % Checks
    cond_max_jump = all(abs(diff_pos) < max_jump);

    % For thetaS < 0 (first half of diffs), magnitude should be decreasing.
    cond_mag_decreasing = all(diff_abs_pos(1:midpoint-1) <= tollerance);
    
    % For thetaS > 0 (from midpoint onwards), magnitude should be increasing.
    cond_mag_increasing = all(diff_abs_pos(midpoint:end) >= -tollerance);

    if cond_max_jump && cond_mag_decreasing && cond_mag_increasing
        validRowCount = validRowCount + 1;
        tempMatrix(validRowCount, :) = currentRow_pos;
    end
end
output_secondpass_pos = tempMatrix(1:validRowCount, :);


% For negative branch
[Rows_neg, Cols_neg] = size(intermediate_neg);
tempMatrix2 = zeros(Rows_neg, Cols_neg);
validRowCount2 = 0;
for rowIndex = 1:Rows_neg
    currentRow_neg = intermediate_neg(rowIndex, :);
    thetaWheel_neg_values = currentRow_neg(num_val+1:end);
    diff_neg = diff(thetaWheel_neg_values);
    diff_abs_neg = diff(abs(thetaWheel_neg_values));

    % Checks
    cond_max_jump = all(abs(diff_neg) < max_jump);

    % For thetaS < 0 (first half of diffs), magnitude should be decreasing.
    cond_mag_decreasing = all(diff_abs_neg(1:midpoint-1) <= tollerance);
    
    % For thetaS > 0 (from midpoint onwards), magnitude should be increasing.
    cond_mag_increasing = all(diff_abs_neg(midpoint:end) >= -tollerance);

    if cond_max_jump && cond_mag_decreasing  && cond_mag_increasing
        validRowCount2 = validRowCount2 + 1;
        tempMatrix2(validRowCount2, :) = currentRow_neg;
    end
end
output_secondpass_neg = tempMatrix2(1:validRowCount2, :);

disp(['Pass 2 complete. Pos cases: ', num2str(size(output_secondpass_pos, 1)), ', Neg cases: ', num2str(size(output_secondpass_neg, 1))]);

% -- Third Pass: Ackermann check --
% For positive branch
[Rows_pos, Cols_pos] = size(output_secondpass_pos);
tempMatrix = zeros(Rows_pos, Cols_pos);
validRowCount = 0;
for rowIndex = 1:Rows_pos
    currentRow_pos = output_secondpass_pos(rowIndex, :);

    %ackermann percentage check between values
    thetaIn = currentRow_pos(thetaIndexFirst);
    thetaOut = currentRow_pos(thetaIndexLast);
    percentAck = ((abs(thetaIn) - abs(thetaOut)) ./ abs(thetaIn)) .* 100;
    
    %check change in ackermann percentages
    cond_percentAck = (percentAck >= ackLimLow) && (percentAck <= ackLimHigh);

    %Turning circle check
    Ds = 2 .* ((wb ./ sind(abs(thetaOut))) + rs);

    if (cond_percentAck)
        validRowCount = validRowCount + 1;
        tempMatrix(validRowCount, :) = currentRow_pos;
    end   
end

output_final_pos = tempMatrix(1:validRowCount, :);

% Repeat for negative branch
[Rows_neg, Cols_neg] = size(output_secondpass_neg);
tempMatrix2 = zeros(Rows_neg, Cols_neg);
validRowCount2 = 0;

for rowIndex = 1:Rows_neg
    currentRow_neg = output_secondpass_neg(rowIndex, :);

    % ackermann percentage check between values
    thetaIn = currentRow_neg(thetaIndexFirst);
    thetaOut = currentRow_neg(thetaIndexLast);
    percentAck = ((abs(thetaIn) - abs(thetaOut)) ./ abs(thetaIn)) .* 100;

    % check change in ackermann percentages
    cond_percentAck = (percentAck >= ackLimLow) && (percentAck <= ackLimHigh);

    % Turning circle check
    Ds = 2 .* ((wb ./ sind(abs(thetaOut))) + rs);

    if cond_percentAck
        validRowCount2 = validRowCount2 + 1;
        tempMatrix2(validRowCount2, :) = currentRow_neg;
    end
end

output_final_neg = tempMatrix2(1:validRowCount2, :);

disp(['Pass 3 complete. Pos cases: ', num2str(size(output_final_pos, 1)), ', Neg cases: ', num2str(size(output_final_neg, 1))]);

% -- Build header row --
thetaWheel_headers = cell(1, num_theta);
    for g = 1:num_theta
        thetaWheel_headers{g} = ['thetaWheel_' num2str(theta_vals(g))];
    end
headers = ["h", "t", "r/2", "zr", "cx", "cz", thetaWheel_headers];

disp('Created Headers')

fid = fopen('steering_pos.csv', 'w');
header_line = strjoin(headers, ',');
fprintf(fid, '%s\n', header_line);

% -- write matrix --
[rows, cols] = size(output_final_pos);
%[rows, cols] = size(output_pos);
%[rows, cols] = size(intermediate_pos);
%[rows, cols] = size(output_secondpass_pos);
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
%[rows, cols] = size(output_neg);
%[rows, cols] = size(intermediate_neg);
%[rows, cols] = size(output_secondpass_neg);

if rows > 0
    fmt = [repmat('%g,', 1, cols-1) '%g\n'];
    for r = 1:rows
        fprintf(fid2, fmt, output_final_neg(r, :));
    end
end
fclose(fid2);

fid3 = fopen('steering_arg.csv', 'w');
header_line = strjoin(headers, ',');
fprintf(fid3, '%s\n', header_line);

[rows, cols] = size(output_arg);
if rows > 0
    fmt = [repmat('%g,', 1, cols-1) '%g\n'];
    for r = 1:rows
        fprintf(fid3, fmt, output_arg(r, :));
    end
end
fclose(fid3);

% fid4 = fopen('steering_R.csv', 'w');
% header_line = strjoin(headers, ',');
% fprintf(fid4, '%s\n', header_line);

% [rows, cols] = size(output_R);
% if rows > 0
%     fmt = [repmat('%g,', 1, cols-1) '%g\n'];
%     for r = 1:rows
%         fprintf(fid4, fmt, output_R(r, :));
%     end
% end
% fclose(fid4);

% fid5 = fopen('steering_T.csv', 'w');
% header_line = strjoin(headers, ',');
% fprintf(fid5, '%s\n', header_line);

% [rows, cols] = size(output_T);
% if rows > 0
%     fmt = [repmat('%g,', 1, cols-1) '%g\n'];
%     for r = 1:rows
%         fprintf(fid5, fmt, output_T(r, :));
%     end
% end
% fclose(fid5);

% fid6 = fopen('steering_phi.csv', 'w');
% header_line = strjoin(headers, ',');
% fprintf(fid6, '%s\n', header_line);

% [rows, cols] = size(output_phi);
% if rows > 0
%     fmt = [repmat('%g,', 1, cols-1) '%g\n'];
%     for r = 1:rows
%         fprintf(fid6, fmt, output_phi(r, :));
%     end
% end
% fclose(fid6);


disp('Print to CSV')

disp('done')
