clear
clc

tic();

sweep = 10;

% -------------------------------------------------------------------------
% INPUT VARIABLES - fill in values before running
% -------------------------------------------------------------------------
m = 28;       % Upper chassis beam
n = 33;       % Lower chassis beam
l_length = 68.64553882081486;% Lower arm length
l_angle = 2.087114971053753; % Lower arm angle (degrees)
u_angle = 6.411892338485681; % Upper arm angle (degrees)
u_length = 67.15904891545478;% Upper arm length
hr = 23;      % Ride height
sumh = 54;    % Frame height from ground
y1 = 20.5;      % Lower upright point height
y2 = 61.5;      % Upper upright point height
% -------------------------------------------------------------------------

deltay = y2 - y1;
l = l_length * cosd(l_angle);
u = u_length * cosd(u_angle);
 
od = 21;
d = 90;
dcompFull = 80;
thetad = 95;
 
% Generate headers for the final format
constant_headers = {'Upper Beam', 'Lower Beam', 'Lower arm length', 'Lower arm angle', 'Upper arm length', 'Upper arm angle', 'Ride', 'Frame_h_from_Ground', 'y1', 'y2', 'WheelTravel'};
dynamic_headers = {};
for i = 0:sweep
    dynamic_headers{end+1} = ['alpha @ theta_', num2str(i)];
end
for i = 0:sweep
    dynamic_headers{end+1} = ['camber @ theta_', num2str(i)];
end
for i = 0:sweep
    dynamic_headers{end+1} = ['test_val @ theta_', num2str(i)];
end
headers = [constant_headers, dynamic_headers];
header_line = strjoin(headers, ',');
 
fid = fopen('front_data_rework_motion.csv', 'w');
if fid == -1
    error('Cannot open file for writing.');
end
fprintf(fid, '%s\n', header_line);
fclose(fid);
 
set1 = 0:1:sweep;
set2 = 0:0.25:sweep;
 
valid_combinations = [];
 
for theta = set1
    initial_alpha = NaN;
    min_dist_error_coarse = inf;
 
    Lx = n + (l.*cosd(theta)) - ((y1 - hr).*sind(theta));
    Ly = hr + (l.*sind(theta)) + ((y1 - hr).*cosd(theta));
 
    for alpha_guess = set2
        Ux = m + (u.*cosd(alpha_guess)) - ((y2 - sumh).*sind(alpha_guess));
        Uy = sumh + (u.*sind(alpha_guess)) + ((y2 - sumh).*cosd(alpha_guess));
 
        distLim = sqrt((Lx - Ux).^2 + (Ly - Uy).^2);
        dist_error = abs(distLim - deltay);
 
        if dist_error < min_dist_error_coarse
            min_dist_error_coarse = dist_error;
            initial_alpha = alpha_guess;
        end
    end
 
    best_alpha = NaN;
    final_dist_error = inf;
 
    if ~isnan(initial_alpha)
        dist_func = @(alpha) sqrt(...
            (Lx - (m + u*cosd(alpha) - (y2 - sumh)*sind(alpha))).^2 + ...
            (Ly - (sumh + u*sind(alpha) + (y2 - sumh)*cosd(alpha))).^2 ...
        ) - deltay;
 
        try
            best_alpha = fzero(dist_func, initial_alpha);
 
            Ux_final = m + (u.*cosd(best_alpha)) - ((y2 - sumh).*sind(best_alpha));
            Uy_final = sumh + (u.*sind(best_alpha)) + ((y2 - sumh).*cosd(best_alpha));
            distLim_final = sqrt((Lx - Ux_final).^2 + (Ly - Uy_final).^2);
            final_dist_error = abs(distLim_final - deltay);
        catch
            best_alpha = initial_alpha;
            final_dist_error = min_dist_error_coarse;
        end
    else
        best_alpha = NaN;
        final_dist_error = inf;
    end
 
    dx = n + ((l - od).*cosd(theta)) - ((y1 - hr).*sind(theta));
    dy = hr + ((l - od).*sind(theta)) + ((y1 - hr).*cosd(theta));
    dxf = (n + l - od) + (d.*(cosd(thetad)));
    dyf = y1 + (d.*(sind(thetad)));
    dcomp = sqrt(((dxf - dx).^2) + ((dyf - dy).^2));
 
    Ux = m + (u.*cosd(best_alpha)) - ((y2 - sumh).*sind(best_alpha));
    Uy = sumh + (u.*sind(best_alpha)) + ((y2 - sumh).*cosd(best_alpha));
 
    grad1 = (Uy - Ly);
    grad2 = (Ux - Lx);
    camber_fac90 = atan2d(grad1, grad2);
    camber = 90 - camber_fac90;
 
    valid_combinations(end+1, :) = [best_alpha, camber, dcomp];
 
end
 
theta_15_index = sweep + 1;
camber_at_15 = valid_combinations(theta_15_index, 2);
camber_at_0 = valid_combinations(1, 2);
dcomp_at_15 = valid_combinations(theta_15_index, 3);
c_gain = abs(camber_at_15) - abs(camber_at_0);
best_alpha_15 = valid_combinations(theta_15_index, 1);
best_alpha_0 = valid_combinations(1, 1);
 
Uy_0 = sumh + (u.*sind(best_alpha_0)) + ((y2 - sumh).*cosd(best_alpha_0));
Uy_15 = sumh + (u.*sind(best_alpha_15)) + ((y2 - sumh).*cosd(best_alpha_15));
WheelTravel = Uy_15 - Uy_0;
 
cond_acceptable = (c_gain < 3) & (dcompFull < dcomp_at_15) & (abs(best_alpha_0) <= 0.5);
 
if ~cond_acceptable
    disp('Configuration does not meet acceptability criteria, no output written.');
else
    num_expected = sweep + 1;
    alphas = zeros(1, num_expected);
    camber_vals = zeros(1, num_expected);
    test_vals = zeros(1, num_expected);
 
    num_combinations = size(valid_combinations, 1);
 
    if num_combinations > 0
        actual_alphas = valid_combinations(:, 1)';
        camber_found = valid_combinations(:, 2)';
        test_found = valid_combinations(:, 3)';
 
        len_to_copy = min(num_combinations, num_expected);
        alphas(1:len_to_copy) = actual_alphas(1:len_to_copy);
        camber_vals(1:len_to_copy) = camber_found(1:len_to_copy);
        test_vals(1:len_to_copy) = test_found(1:len_to_copy);
 
        if num_combinations > num_expected
            fprintf('Warning: Found %d valid combinations, but space for only %d. Truncating.\n', num_combinations, num_expected);
        end
    end
 
    constants_row = [m, n, l_length, l_angle, u_length, u_angle, hr, sumh, y1, y2, WheelTravel];
    output_row = [constants_row, alphas, camber_vals, test_vals];
 
    dlmwrite('front_data_rework_motion.csv', output_row, '-append', 'delimiter', ',');
end
 
endtime = toc();
disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])
