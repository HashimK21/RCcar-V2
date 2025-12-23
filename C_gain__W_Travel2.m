clear
clc

tic();

%pull values from csv
values_data = dlmread('rear_data.csv', ',', 2, 0);
%values_data = dlmread('front_data.csv', ',', 2, 0); %pull values from front

sweep = 15;

% Generate headers for the final format
constant_headers = {'m', 'n', 'l', 'u', 'hr', 'sumh', 'y1', 'y2'};
dynamic_headers = {};
for i = 0:sweep
    dynamic_headers{end+1} = ['theta_', num2str(i)];
end
for i = 0:sweep
    dynamic_headers{end+1} = ['alpha_', num2str(i)];
end
headers = [constant_headers, dynamic_headers];
header_line = strjoin(headers, ',');

% Write headers to a clean file
fid = fopen('c_gain_rear.csv', 'w');
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

    %od = 40; %for front
    od = 10; %for rear

    d = 110; %damper length
    dcompFull = 90; %length of fully compressed damper

    thetad = 110; %angle of damper anticlockwise from x axis

    %set theta and alpha (angles for arm rotation)
    set1 = 0:1:sweep;
    set2 = 0:0.25:sweep;

    valid_combinations = [];

    for theta = set1
        best_alpha = NaN;
        min_dist_error = inf;

        for alpha = set2
            % All calculations from your original loop are preserved here.
            %lower arm rotaion
            Lx = n + (l.*cosd(theta)) - ((y1 - hr).*sind(theta));
            Ly = hr + (l.*sind(theta)) + ((y1 - hr).*cosd(theta));

            %upper arm rotation
            Ux = m + (u.*cosd(alpha)) - ((y2 - sumh).*sind(alpha));
            Uy = sumh + (u.*sind(alpha)) + ((y2 - sumh).*cosd(alpha));

            %damper base rotation
            dx = n + ((l - od).*cosd(theta)) - ((y1 - hr).*sind(theta)); %x rotation
            dy = hr + ((l - od).*sind(theta)) + ((y1 - hr).*cosd(theta)); %y rotation

            dxf = (n + l - od) + (d.*(sind(thetad)));
            dyf = y1 + (d.*(cosd(thetad)));

            %rotaion limitations
            distLim = sqrt((Lx - Ux).^2 + (Ly - Uy).^2);
            dcomp = sqrt(((dxf - dx).^2) + ((dyf - dy).^2));

            dist_error = abs(distLim - deltay);

            % We find the alpha that gives the minimum distance error.
            if dist_error < min_dist_error
                min_dist_error = dist_error;
                best_alpha = alpha;
            end
        end

        % After checking all alphas, we save the one that had the smallest error.
        % This guarantees we get exactly one alpha for each of the 16 thetas.

        % <<< YOUR FURTHER CALCULATION USING 'theta' AND 'best_alpha' WOULD GO HERE >>>
        

        valid_combinations(end+1, :) = [theta, best_alpha];
    end

    % Determine the expected number of columns for thetas and alphas from 'sweep'
    num_expected = sweep + 1;

    % Initialize theta and alpha arrays with zeros, which will be used for padding
    thetas = zeros(1, num_expected);
    alphas = zeros(1, num_expected);

    num_combinations = size(valid_combinations, 1);

    if num_combinations > 0
        % Get the actual combinations found
        actual_thetas = valid_combinations(:, 1)';
        actual_alphas = valid_combinations(:, 2)';

        % Determine how many values to copy over (handles both padding and truncation)
        len_to_copy = min(num_combinations, num_expected);

        thetas(1:len_to_copy) = actual_thetas(1:len_to_copy);
        alphas(1:len_to_copy) = actual_alphas(1:len_to_copy);

        if num_combinations > num_expected
            fprintf('Warning: Found %d valid combinations, but space for only %d. Truncating for input row %d.\n', num_combinations, num_expected, row_idx);
        end
    end

    % Construct the full row for the CSV
    constants_row = [m, n, l_length, u_length, hr, sumh, y1, y2];
    output_row = [constants_row, thetas, alphas];

    % Append the data row to the CSV, ensuring a consistent number of columns
    dlmwrite('c_gain_rear.csv', output_row, '-append', 'delimiter', ',');

end

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])
