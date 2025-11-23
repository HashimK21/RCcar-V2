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
set2 = 10:5:30; %values for steering arm, s
set3 = 50:10:100; %values for tie rod, t
set4 = 30:5:50; %values for r/2
set5 = 10:10:100; %values for z1

[set1, set2, set3, set4, set5] = ndgrid(set1, set2, set3, set4, set5);
combinations = [set1(:), set2(:), set3(:), set4(:), set5(:)];

%theta_vals = linspace((-pi/4), (pi/4), 90);
theta_vals = linspace(0, (pi/4), 45);

num_cases = numel(set1);
num_theta = numel(theta_vals);
num_val = 5;

% Preallocate output matrix
output = zeros(num_cases, num_val + num_theta);
test = zeros(num_cases, num_val + num_theta);

disp('created ngrid and output matrix')

for i = 1:numel(set1)
    h = set1(i);
    s = set2(i);
    t = set3(i);
    rO2 = set4(i);
    z1 = set5(i);

    %h = 40;
    %s = 20;
    %t = 80;
    %rO2 = 40;
    %z1 = 40;
    thetaWheel_vals = zeros(1, num_theta);
    xr_vals =  zeros(1, num_theta);
    parfor p = 1:numel(theta_vals)
        thetaS = theta_vals(p);
        %thetaS = 0;
        %finding rack displacement
        Delx = h .* sin(thetaS);

        xr = Delx + rO2;

        %Wheel angle
        xp = xj - k; %x value for wheel pivot
        zp = 0;

        cx = 15; %distance to wheel pivot from steering arm tie rod joint, x
        cz = -(20 + s); %distance to wheel pivot from steering arm tie rod joint, z

        dx = xp - xr;
        dz = zp - z1;

        Tnum = (t^2) - ((cx)^2) - ((cz)^2) - ((dx)^2) - ((dz)^2);
        T = Tnum ./ 2; %constant terms of each case

        alpha = (dx .* cx) + (dz .* cz);
        beta = (dz .* cx) + (dx .* cz);

        dom = sqrt(((alpha)^2) + ((beta)^2));

        thetaWheelFromHorz = acos((T./dom)) + atan((beta)./ (alpha));

        thetaWheelRAD = ((pi)/2) - thetaWheelFromHorz;

        thetaWheel = thetaWheelRAD .* (180/pi);
        %thetaWheel = thetaWheelFromHorz .* (180/pi);

        thetaWheel_vals(p) = thetaWheel;   % store output
        xr_vals(p) = xr;

    end

        % One row per case: 5 constants + values for angle dependants
        output(i, :) = [h, s, t, z1, rO2, thetaWheel_vals];
        test(i, :) = [h, s, t, z1, rO2, xr_vals];

end

disp('finished calculations')

%Sort
[numRows, numCols] = size(output);
tempMatrix = zeros(numRows, numCols);  % temporary storage
validRowCount = 0;

for rowIndex = 1:numRows
    if all(imag(output(rowIndex, :)) == 0)   % row has only real numbers
        validRowCount = validRowCount + 1;
        tempMatrix(validRowCount, :) = output(rowIndex, :);
    end
end


output = tempMatrix(1:validRowCount, :);

%further sort

[newRows, newCols] = size(output);
tempMatrix2 = zeros(newRows, newCols);
validRowCount2 = 0;

[~, idxZero] = min(abs(theta_vals - 0)); % location of thetaS = 0
thetaIndex = num_val + idxZero; % the correct column in output
thetaIndexLast = num_val + num_theta;

for rowIndex = 1:newRows
    currentRow = output(rowIndex, :);

    %conditions for acceptance
    cond_exist_real = isreal(xr);
    cond_theta0_under_1 = currentRow(thetaIndex) < 1;
    cond_theta0_over_0 = currentRow(thetaIndex) > 0;
    cond_thetaLast_neg = currentRow(thetaIndexLast) < 0;



    if cond_exist_real && cond_theta0_under_1 && cond_theta0_over_0 && cond_thetaLast_neg
       validRowCount2 = validRowCount2 + 1;
       tempMatrix2(validRowCount2, :) = currentRow;
    end
end

output = tempMatrix2(1:validRowCount2, :);

disp('Completed Purge')

%Build header row
thetaWheel_headers = cell(1, num_theta);
    for g = 1:num_theta
        thetaWheel_headers{g} = ['thetaWheel_' num2str(g)];
    end
headers = ["h", "s", "t", "z1", "r/2", thetaWheel_headers];

disp('Created Headers')

%Write header + data
fid = fopen('steering.csv', 'w');
fprintf(fid, '%s,', headers{1:end-1});
fprintf(fid, '%s\n', headers{end});
fclose(fid);

%Append the matrix
dlmwrite('steering.csv', output, '-append');

disp('Print to CSV')

%test cases
[numRows_test, numCols_test] = size(test);
tempMatrix_test = zeros(numRows_test, numCols_test);  % temporary storage
validRowCount_test = 0;

for rowIndex_test = 1:numRows_test
    if all(imag(test(rowIndex_test, :)) == 0)   % row has only real numbers
        validRowCount_test = validRowCount_test + 1;
        tempMatrix_test(validRowCount_test, :) = test(rowIndex_test, :);
    end
end

test = tempMatrix_test(1:validRowCount_test, :);

%Build header row for tests
test_headers = cell(1, num_theta);
    for g_test = 1:num_theta
        test_headers{g_test} = ['testedVariable_' num2str(g_test)];
    end
headers_test = ["h", "s", "t", "z1", "r/2", test_headers];

disp('Created Test Headers')

%Write header + data
fid = fopen('xr_test', 'w');
fprintf(fid, '%s,', headers_test{1:end-1});
fprintf(fid, '%s\n', headers_test{end});
fclose(fid);

%Append test matrix
dlmwrite('xr_test.csv', test, '-append');

disp('Print Test to CSV')

disp('done')
