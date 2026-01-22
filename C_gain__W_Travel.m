clear
clc

tic();

%pull values from csv
%values_data = dlmread('rear_data.csv', ',', 2, 0);
values_data = dlmread('front_data.csv', ',', 2, 0); %pull values from front

sweep = 10;

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

%Write headers to a clean file
% fid = fopen('c_gain_rear.csv', 'w');
% if fid == -1
%     error('Cannot open file for writing.');
% end
% fprintf(fid, '%s\n', header_line);
% fclose(fid);

fid = fopen('c_gain_front.csv', 'w');
if fid == -1
    error('Cannot open file for writing.');
end
fprintf(fid, '%s\n', header_line);
fclose(fid);

for row_idx = 1:size(values_data, 1)
    values = values_data(row_idx, :);

    if length(values) < 17
        fprintf('Skipping row %d due to insufficient columns.\n', row_idx);
        continue;
    end

    m = values(3);
    n = values(4);
    l_length = values(17);
    l_angle = values(16);
    l = l_length * cosd(l_angle);
    u_angle = values(14);
    u_length = values(15);
    u = u_length * cosd(u_angle);
    hr = values(5);
    sumh = values(7);
    y1 = values(8);
    y2 = values(9);

    deltay = y2 - y1;

    od = 35; %for front
    %od = 10; %for rear

    d = 90; %damper length
    dcompFull = 80; %length of fully compressed damper

    thetad = 110; %angle of damper anticlockwise from x axis

    %set theta and alpha (angles for arm rotation)
    set1 = 0:1:sweep;
    set2 = 0:0.25:sweep;

    valid_combinations = [];

    for theta = set1
        % --- Coarse search to find a good starting point for the solver ---
        initial_alpha = NaN;
        min_dist_error_coarse = inf;

        % Lower arm rotation is constant for the inner loop
        Lx = n + (l.*cosd(theta)) - ((y1 - hr).*sind(theta));
        Ly = hr + (l.*sind(theta)) + ((y1 - hr).*cosd(theta));

        for alpha_guess = set2
            % Upper arm rotation
            Ux = m + (u.*cosd(alpha_guess)) - ((y2 - sumh).*sind(alpha_guess));
            Uy = sumh + (u.*sind(alpha_guess)) + ((y2 - sumh).*cosd(alpha_guess));
            
            % Rotation limitations
            distLim = sqrt((Lx - Ux).^2 + (Ly - Uy).^2);
            dist_error = abs(distLim - deltay);

            if dist_error < min_dist_error_coarse
                min_dist_error_coarse = dist_error;
                initial_alpha = alpha_guess;
            end
        end
        % --- End of coarse search ---

        best_alpha = NaN;
        final_dist_error = inf;

        if ~isnan(initial_alpha)
            % Define the function to solve: we want distLim - deltay = 0
            dist_func = @(alpha) sqrt(...
                (Lx - (m + u*cosd(alpha) - (y2 - sumh)*sind(alpha))).^2 + ...
                (Ly - (sumh + u*sind(alpha) + (y2 - sumh)*cosd(alpha))).^2 ...
            ) - deltay;

            try
                % Use fzero to find the precise alpha that makes dist_func zero.
                best_alpha = fzero(dist_func, initial_alpha);
                
                % Recalculate the final distance error with the precise alpha
                Ux_final = m + (u.*cosd(best_alpha)) - ((y2 - sumh).*sind(best_alpha));
                Uy_final = sumh + (u.*sind(best_alpha)) + ((y2 - sumh).*cosd(best_alpha));
                distLim_final = sqrt((Lx - Ux_final).^2 + (Ly - Uy_final).^2);
                final_dist_error = abs(distLim_final - deltay);
            catch
                % If solver fails, fall back to the best result from the coarse search
                best_alpha = initial_alpha;
                final_dist_error = min_dist_error_coarse;
            end
        else
            % Coarse search failed to find any valid alpha, store placeholders
            best_alpha = NaN;
            final_dist_error = inf;
        end

        % Damper base rotation is also constant for the inner loop
        dx = n + ((l - od).*cosd(theta)) - ((y1 - hr).*sind(theta)); %x rotation
        dy = hr + ((l - od).*sind(theta)) + ((y1 - hr).*cosd(theta)); %y rotation
        dxf = (n + l - od) + (d.*(cosd(thetad)));
        dyf = y1 + (d.*(sind(thetad)));
        dcomp = sqrt(((dxf - dx).^2) + ((dyf - dy).^2));


        % gradient for tan function for camber angle
        
        % Upper arm rotation recalc with best alpha
        Ux = m + (u.*cosd(best_alpha)) - ((y2 - sumh).*sind(best_alpha));
        Uy = sumh + (u.*sind(best_alpha)) + ((y2 - sumh).*cosd(best_alpha));
        
        grad1 = (Uy - Ly);
        grad2 = (Ux - Lx);
        camber_fac90 = atan2d(grad1, grad2);
        camber = 90 - camber_fac90;
 
        valid_combinations(end+1, :) = [best_alpha, camber, dcomp];

    end

    theta_15_index = sweep + 1; % coresponds to theta = 15
    camber_at_15 = valid_combinations(theta_15_index, 2);
    camber_at_0 = valid_combinations(1, 2);
    dcomp_at_15 = valid_combinations(theta_15_index, 3);
    c_gain = abs(camber_at_15) - abs(camber_at_0);
    best_alpha_15 = valid_combinations(theta_15_index, 1);
    best_alpha_0 = valid_combinations(1, 1);

    Uy_0 = sumh + (u.*sind(best_alpha_0)) + ((y2 - sumh).*cosd(best_alpha_0));
    Uy_15 = sumh + (u.*sind(best_alpha_15)) + ((y2 - sumh).*cosd(best_alpha_15));
    WheelTravel = Uy_15 - Uy_0;

    cond_acceptable =  (c_gain < 3) & (dcompFull < dcomp_at_15) & (abs(best_alpha_0) <= 0.5); %& (abs(dcomp - dcompFull) > 0.1) & (abs(dcomp - dcompFull) <= 0.4);
    
    if ~cond_acceptable
        continue; % Skip this entire data row if condition not met
    end

    % Determine the expected number of columns for thetas and alphas from 'sweep'
    num_expected = sweep + 1;

    % Initialize theta and alpha arrays with zeros, which will be used for padding
    alphas = zeros(1, num_expected);
    camber_vals = zeros(1, num_expected);
    test_vals = zeros(1, num_expected);

    num_combinations = size(valid_combinations, 1);

    if num_combinations > 0
        % Get the actual combinations found
        actual_alphas = valid_combinations(:, 1)';
        camber_found = valid_combinations(:, 2)';
        test_found = valid_combinations(:, 3)';

        % Determine how many values to copy over (handles both padding and truncation)
        len_to_copy = min(num_combinations, num_expected);

        alphas(1:len_to_copy) = actual_alphas(1:len_to_copy);
        camber_vals(1:len_to_copy) = camber_found(1:len_to_copy);
        test_vals(1:len_to_copy) = test_found(1:len_to_copy);

        if num_combinations > num_expected
            fprintf('Warning: Found %d valid combinations, but space for only %d. Truncating for input row %d.\n', num_combinations, num_expected, row_idx);
        end
    end

    % Construct the full row for the CSV
    constants_row = [m, n, l_length, l_angle, u_length, u_angle, hr, sumh, y1, y2, WheelTravel];
    output_row = [constants_row, alphas, camber_vals, test_vals];

    % Append the data row to the CSV, ensuring a consistent number of columns
    %dlmwrite('c_gain_rear.csv', output_row, '-append', 'delimiter', ',');
    dlmwrite('c_gain_front.csv', output_row, '-append', 'delimiter', ',');

end

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])
