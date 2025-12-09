clc

%global constants
tw = 220; %track width
wb = 352; %wheelbase
tireD = 85; %tire diamater
tireW = 34; %tire width
k = -6.93; %offset for KPI mid point projection
rs = 4.97; %scrub radius

xj = (tw/2) - (tireW/2); %tire wall, used to place sus joints

%-- Create ndgrid of variables --
set1 = 10:1:150; %values for tie rod, t
set2 = 10:1:100; %values for r/2

[set1, set2] = ndgrid(set1, set2);
combinations = [set1(:), set2(:)];

%steering angle in radians
ServoAngle = 25;

StepSize = (2*ServoAngle) + 1;
theta_vals = linspace(-(ServoAngle), ServoAngle, StepSize);


num_cases = numel(set1);
num_theta = numel(theta_vals);
num_val = 6; %number of ngrid variables + constants

% Preallocate output matrix
output_pos = zeros(num_cases, num_val + num_theta);
output_neg = zeros(num_cases, num_val + num_theta);

disp('created ndgrid and output matrix')

tic();

for i = 1:numel(set1)
    t = set1(i);
    rO2 = set2(i);
    
    h = 20; %servo arm length
    cxComp = 12; %steering lever x component
    czComp = 10; %steering lever z component

    %Wheel Pivot Position
    xp = xj - k; %x towards outside of car is positive
    zp = 0; %z towards rear of car is negative

    thetaWheel_vals_pos = zeros(1, num_theta);
    thetaWheel_vals_neg = zeros(1, num_theta);
    for p = 1:numel(theta_vals)
        thetaS = theta_vals(p);
        %thetaS = 0;
        %finding rack displacement
        Delx = h .* sind(thetaS);
        xr = Delx + rO2;

        cx = -(cxComp); %distance to wheel pivot from steering arm tie rod joint, x
        cz = -(czComp); %distance to wheel pivot from steering arm tie rod joint, z

        %Finding zr position
        zr_pos_branch = zp + cz + sqrt((t^2) - ((xp + cx - rO2)^2));
        zr_neg_branch = zp + cz - sqrt((t^2) - ((xp + cx - rO2)^2));

        if (zr_neg_branch < 0) && (zr_pos_branch >= 0)
            zr = zr_neg_branch;

        elseif (zr_pos_branch < 0) && (zr_neg_branch >= 0)
            zr = zr_pos_branch;

        else
            zr = NaN;

        end

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
            thetaWheelRAD = NaN;
        else
            arg = T ./ R;
            arg = min(max(arg, -1), 1); % clamp to [-1,1]
            phi = atan2(beta, alpha); % correct quadrant
            thetaWheelRAD_pos = phi + acos(arg);
            thetaWheelRAD_neg = phi - acos(arg);
        end

        thetaWheel_pos = thetaWheelRAD_pos .* (180/pi);
        thetaWheel_neg = thetaWheelRAD_neg .* (180/pi);

        thetaWheel_vals_pos(p) = thetaWheel_pos;   % store output for positive acos
        thetaWheel_vals_neg(p) = thetaWheel_neg;   % store output for negative acos

    end

        % One row per case: 5 constants + values for angle dependants
        output_pos(i, :) = [h, t, rO2, zr, cx, cz, thetaWheel_vals_pos];
        output_neg(i, :) = [h, t, rO2, zr, cx, cz, thetaWheel_vals_neg];

end

end_time = toc();
disp(['finished calculations in time:' num2str(end_time) ' seconds'])

% -- Purge bad rows --
%location of thetaS = 0
[~, idxZero] = min(abs(theta_vals));
thetaIndex = num_val + idxZero;
thetaIndexLast = num_val + num_theta;

%First pass variables
max_upper_lim = 0.01;
%Second pass variables
max_jump = 5; % Max allowable jump between thetaWheel values in degrees
%Third pass variables
ackLim = 20; %min ackermann percentage
TurningCircleLim = 300; %min turning circle diameter in mm


% -- First Pass: Filter based on thetaIndex and thetaIndexLast conditions --
cond_pos_pass1 = (abs(output_pos(:, thetaIndex)) <= max_upper_lim) ...
                 & (abs(output_pos(:, thetaIndex)) >= 0) ...
                 & (output_pos(:, thetaIndexLast) < 0) ...
                 & (output_pos(:, num_val + 1) > 0); 
intermediate_pos = output_pos(cond_pos_pass1, :);

cond_neg_pass1 = (abs(output_neg(:, thetaIndex)) <= max_upper_lim) ...
                 & (abs(output_neg(:, thetaIndex)) >= 0) ...
                 & (output_neg(:, thetaIndexLast) < 0) ...
                 & (output_pos(:, num_val + 1) > 0);
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
    cond_mag_decreasing = all(diff_abs_pos(1:ceil(num_theta/2)-1) <= 0);
    
    % For thetaS > 0 (from midpoint onwards), magnitude should be increasing.
    cond_mag_increasing = all(diff_abs_pos(ceil(num_theta/2):end) >= 0);

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
    cond_mag_decreasing = all(diff_abs_neg(1:ceil(num_theta/2)-1) <= 0);
    
    % For thetaS > 0 (from midpoint onwards), magnitude should be increasing.
    cond_mag_increasing = all(diff_abs_neg(ceil(num_theta/2):end) >= 0);

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
    first_val = num_val+1;
    last_val = num_val + num_theta;
    
    %ackermann percentage check between values
    thetaIn = currentRow_pos(first_val);
    thetaOut = currentRow_pos(last_val);
    percentAck = ((abs(thetaIn) - abs(thetaOut)) ./ abs(thetaIn)) .* 100;
    
    %check change in ackermann percentages
    cond_percentAck = (percentAck >= ackLim);

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
    first_val = num_val+1;
    last_val = num_val + num_theta;

    % ackermann percentage check between values
    thetaIn = currentRow_pos(first_val);
    thetaOut = currentRow_pos(last_val);
    percentAck = ((abs(thetaIn) - abs(thetaOut)) ./ abs(thetaIn)) .* 100;

    % check change in ackermann percentages
    cond_percentAck = (percentAck >= ackLim);

    % Turning circle check
    Ds = 2 .* ((wb ./ sind(abs(thetaOut))) + rs);

    if cond_percentAck
        validRowCount2 = validRowCount2 + 1;
        tempMatrix2(validRowCount2, :) = currentRow_neg;
    end
end

output_final_neg = tempMatrix2(1:validRowCount2, :);

disp(['Pass 3 complete. Final pos cases: ', num2str(size(output_final_pos, 1)), ', Final neg cases: ', num2str(size(output_final_neg, 1))]);

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
