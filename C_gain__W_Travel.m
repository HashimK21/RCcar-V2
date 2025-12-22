clear
clc

tic();

clear
clc

tic();

%pull values from csv
values_data = dlmread('rear_data.csv', ',', 2, 0);
%values_data = dlmread('front_data.csv', ',', 2, 0); %pull values from front

all_results = {};

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

    % --- Print distLim at theta=0, alpha=0 for current row ---
    theta_zero = 0;
    alpha_zero = 0;
    Lx_zero = n + (l.*cosd(theta_zero)) - ((y1-hr).*sind(theta_zero));
    Ly_zero = hr + (l.*sind(theta_zero)) + ((y1-hr).*cosd(theta_zero));
    Ux_zero = m + (u.*cosd(alpha_zero)) - ((y2-sumh).*sind(alpha_zero));
    Uy_zero = sumh + (u.*sind(alpha_zero)) + ((y2-sumh).*cosd(alpha_zero));
    distLim_at_zero = sqrt((Lx_zero - Ux_zero).^2 + (Ly_zero - Uy_zero).^2);
    fprintf('CSV Row %d: distLim at theta=0, alpha=0 is %f\n', row_idx, distLim_at_zero);
    % --- End of print block ---

    %od = 40; %for front
    od = 10; %for rear

    d = 110; %damper length
    dcompFull = 90; %length of fully compressed damper

    thetad = 110; %angle of damper anticlockwise from x axis

    %set theta and alpha (angles for arm rotation)
    set1 = 0:1:15;
    set2 = 0:0.25:15;

    [theta_grid, alpha_grid] = ndgrid(set1, set2);
    
    valid_combinations = [];

    for i = 1:numel(theta_grid)
        theta = theta_grid(i);
        alpha = alpha_grid(i);

        %lower arm rotaion
        Lx = n + (l.*cosd(theta)) - ((y1-hr).*sind(theta));
        Ly = hr + (l.*sind(theta)) + ((y1-hr).*cosd(theta));

        %upper arm rotation
        Ux = m + (u.*cosd(alpha)) - ((y2-sumh).*sind(alpha));
        Uy = sumh + (u.*sind(alpha)) + ((y2-sumh).*cosd(alpha));

        %damper base rotation
        dx = n + ((l - od).*cosd(theta)) - ((y1-hr).*sind(theta)); %x rotation
        dy = hr + ((l - od).*sind(theta)) + ((y1-hr).*cosd(theta)); %y rotation

        dxf = (n + l - od) + (d.*(sind(thetad)));
        dyf = y1 + (d.*(cosd(thetad)));

        %rotaion limitations
        distLim = sqrt((Lx - Ux).^2 + (Ly - Uy).^2);
        dcomp = sqrt(((dxf - dx).^2) + ((dyf - dy).^2));

        %tolerances for checks
        tol_dist = 1;

        %Filter valid combinations
        is_close = abs(distLim - deltay) <= tol_dist;
        is_ok_dcomp = (dcomp >= dcompFull) & (dcomp <= d);
        
        if is_close && is_ok_dcomp
            valid_combinations(end+1, :) = [theta, alpha];
        end
    end
    
    if ~isempty(valid_combinations)
        num_valid = size(valid_combinations, 1);
        original_values = repmat([m, n, l, u, hr, sumh, y1, y2], num_valid, 1);
        result_row = [original_values, valid_combinations];
        all_results{end+1} = result_row;
    end

end

final_matrix = cat(1, all_results{:});

headers = {'m', 'n', 'l', 'u', 'hr', 'sumh', 'y1', 'y2', 'valid_theta', 'valid_alpha'};

fid = fopen('c_gain_rear.csv', 'w');
if fid == -1
    error('Cannot open file for writing.');
end

header_line = strjoin(headers, ',');
fprintf(fid, '%s\n', header_line);

if ~isempty(final_matrix)
    writematrix(final_matrix, 'c_gain_rear.csv', 'WriteMode', 'append');
end

fclose(fid);

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])
