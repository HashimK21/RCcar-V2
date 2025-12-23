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
    
    % START of requested debugging block for row 6
    if row_idx == 6
        disp('--- Debugging Info for Row 6 ---');

        % --- Values at theta 0 ---
        theta_0 = 0;
        % Find the best_alpha for theta=0 from the results
        alpha_at_0 = valid_combinations(valid_combinations(:,1) == theta_0, 2);
        
        % Recalculate values for theta=0 and its best_alpha
        Lx_0 = n + (l.*cosd(theta_0)) - ((y1-hr).*sind(theta_0));
        Ly_0 = hr + (l.*sind(theta_0)) + ((y1-hr).*cosd(theta_0));
        Ux_0 = m + (u.*cosd(alpha_at_0)) - ((y2-sumh).*sind(alpha_at_0));
        Uy_0 = sumh + (u.*sind(alpha_at_0)) + ((y2-sumh).*cosd(alpha_at_0));
        distLim_0 = sqrt((Lx_0 - Ux_0).^2 + (Ly_0 - Uy_0).^2);
        
        dx_0 = n + ((l - od).*cosd(theta_0)) - ((y1-hr).*sind(theta_0));
        dy_0 = hr + ((l - od).*sind(theta_0)) + ((y1-hr).*cosd(theta_0));
        dxf_0 = (n + l - od) + (d.*(sind(thetad)));
        dyf_0 = y1 + (d.*(cosd(thetad)));
        decomp_0 = sqrt(((dxf_0 - dx_0).^2) + ((dyf_0 - dy_0).^2));
        
        fprintf('distLim at theta 0: %f\n', distLim_0);
        fprintf('decomp at theta 0: %f\n', decomp_0);
        
        % --- distLim at theta 10 ---
        theta_10 = 10;
        % Find the best_alpha for theta=10 from the results
        alpha_at_10 = valid_combinations(valid_combinations(:,1) == theta_10, 2);
        
        % Recalculate distLim for theta=10 and its best_alpha
        Lx_10 = n + (l.*cosd(theta_10)) - ((y1-hr).*sind(theta_10));
        Ly_10 = hr + (l.*sind(theta_10)) + ((y1-hr).*cosd(theta_10));
        Ux_10 = m + (u.*cosd(alpha_at_10)) - ((y2-sumh).*sind(alpha_at_10));
        Uy_10 = sumh + (u.*sind(alpha_at_10)) + ((y2-sumh).*cosd(alpha_at_10));
        distLim_10 = sqrt((Lx_10 - Ux_10).^2 + (Ly_10 - Uy_10).^2);
        fprintf('distLim at theta 10: %f\n', distLim_10);
        
        % --- decomp at theta 15 and best_alpha for that row ---
        theta_15 = 15;
        % Find the best_alpha for theta=15 from the results
        alpha_at_15 = valid_combinations(valid_combinations(:,1) == theta_15, 2);
        
        % Recalculate decomp for theta=15
        dx_15 = n + ((l - od).*cosd(theta_15)) - ((y1-hr).*sind(theta_15));
        dy_15 = hr + ((l - od).*sind(theta_15)) + ((y1-hr).*cosd(theta_15));
        dxf_15 = (n + l - od) + (d.*(sind(thetad)));
        dyf_15 = y1 + (d.*(cosd(thetad)));
        decomp_15 = sqrt(((dxf_15 - dx_15).^2) + ((dyf_15 - dy_15).^2));
        fprintf('decomp at theta 15 is: %f, with best_alpha: %f\n', decomp_15, alpha_at_15);
        
        disp('--- End of Debugging Info ---');
    end
    % END of requested debugging block
    
    if size(valid_combinations, 1) == 16
        thetas = valid_combinations(:, 1)'; % Transpose to a 1x16 row vector
        alphas = valid_combinations(:, 2)'; % Transpose to a 1x16 row vector
        constants_row = [m, n, l_length, u_length, hr, sumh, y1, y2];
        output_row = [constants_row, thetas, alphas];
        
        % Append the data row to the CSV using an Octave-compatible function
        dlmwrite('c_gain_rear.csv', output_row, '-append', 'delimiter', ',');
    else
        fprintf('Warning: Did not find 16 valid combinations for input row %d. Skipping output for this row.\n', row_idx);
    end

end

endtime = toc();

disp(['Done, Time taken: ' num2str(endtime) ' seconds.'])
