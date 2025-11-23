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
set1 = 10:5:30; %values for a, servo arm length
set2 = 10:5:40; %values for b, link arm length
set3 = 1:1:10; %values for alpha, angle of linkage to normal in neutral position
set4 = 10:5:30; %values for steering arm, s
set5 = 10:10:100; %values for tie rod, t
set6 = 20:5:40; %values for r/2

[set1, set2, set3, set4, set5, set6] = ndgrid(set1, set2, set3, set4, set5, set6);
combinations = [set1(:), set2(:), set3(:), set4(:), set5(:), set6(:)];

theta_vals = linspace(0, 45, 45);

num_cases = numel(set1);
num_theta = numel(theta_vals);
num_val = 6;

% Preallocate output matrix
output = zeros(num_cases, num_val + num_theta);

disp('created ngrid and output matrix')

for i = 1:numel(set1)
    a = set1(i);
    b = set2(i);
    alphao = set3(i);
    s = set4(i);
    t = set5(i);
    rO2 = set6(i);
    thetaWheel_vals = zeros(1, num_theta);  % 90 thetaWheel results
    parfor p = 1:numel(theta_vals)
        thetaS = theta_vals(p);

        %finding rack displacement

        alphab = b .* sind(alphao); %servo offset
        theta1 = alphab + (a .* sind(thetaS));
        theta2 = a .* cosd(thetaS);
        z1 = a + (b .* cosd(alphao));
        N = ((alphab)^2) + (b^2) - ((z1)^2) - ((theta1)^2) - ((theta2)^2); %constants in each case
        L = 2 .* (alphab - theta1);

        SQ = (4 .* ((theta2)^2)) + L;
        X = ((8 .* alphab) .* ((theta2)^2)) + (2 .* N .* L);
        CON = ((4 .* ((theta2)^2)) .* (((alphab)^2) - ((z1)^2))) + (N^2);

        Delx = (X + sqrt((X^2) - (4 .* SQ .* CON))) ./ (2 .* SQ);
        Delxcheck = (X - sqrt((X^2) - (4 .* SQ .* CON))) ./ (2 .* SQ);

        xr = Delx + rO2;
        xrcheck = Delxcheck + rO2;

        zr = ((L .* xr) - N) ./ (2 .* (theta2));
        zrcheck = ((L .* xrcheck) - N) ./ (2 .* (theta2));

        %angle at wheel

        xp = xj - k; %x value for wheel pivot
        zp = 64; %z value for wheel pivot, found by 264 - 190
                 %(3/4 wheelbase, from spring stiffness - servo position from CG - see weightplan.xlsx)

        cx = 15; %distance to wheel pivot from steering arm tie rod joint, x
        cz = 20 + s; %distance to wheel pivot from steering arm tie rod joint, z

        dx = xp - xr;
        dxcheck = xp - xrcheck;
        dz = zp - zr;
        dzcheck = zp - zrcheck;

        Tnum = (t^2) - ((cx)^2) - ((cz)^2) - ((dx)^2) - ((dz)^2);
        T = Tnum ./ 2; %constant terms of each case

        alpha = (dx .* cx) + (dz .* cz);
        beta = (dz .* cx) + (dx .* cz);

        dom = sqrt(((alpha)^2) + ((beta)^2));

        thetaWheelFromHorz = acos((T./dom)) + atan((beta)./ (alpha));

        thetaWheelRAD = ((pi)/2) - thetaWheelFromHorz;

        %thetaWheel = thetaWheelRAD .* (180/pi);
        thetaWheel = thetaWheelFromHorz .* (180/pi);

        %thetaWheel_vals(p) = thetaWheel;   % store output

        %check
        Tnumcheck = (t^2) - ((cx)^2) - ((cz)^2) - ((dxcheck)^2) - ((dzcheck)^2);
        Tcheck = Tnumcheck ./ 2; %constant terms of each case

        alphacheck = (dxcheck .* cx) + (dzcheck .* cz);
        betacheck = (dzcheck .* cx) + (dxcheck .* cz);

        domcheck = sqrt(((alphacheck)^2) + ((betacheck)^2));

        thetaWheelFromHorzcheck = acos((Tcheck./domcheck)) + atan((betacheck)./ (alphacheck));

        thetaWheelRADcheck = ((pi)/2) - thetaWheelFromHorzcheck;

        %thetaWheelcheck = thetaWheelRADcheck .* (180/pi);
        thetaWheelcheck =  thetaWheelFromHorzcheck .* (180/pi);

        thetaWheel_vals(p) = thetaWheelcheck;   % store output

        % One row per case: 5 constants + 90 alpha values
        output(i, :) = [a, b, s, t, z1, rO2, thetaWheel_vals];
    end
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

for rowIndex = 1:newRows
    currentRow = output(rowIndex, :);

    %conditions for acceptance
    cond_exist_real = isreal(xr) && isreal(xrcheck) && ~isnan(xrcheck) && ~isnan(xrcheck);
    cond_nonzero = (xr != 0) && (xrcheck != 0);
    cond_distinct = (xr != xrcheck);
    %cond_rackpos = abs(zr - z1) <= 0.1;
    %cond_rackpos = abs(zrcheck - z1) <= 0.1;

    if cond_exist_real && cond_nonzero && cond_distinct% && cond_rackpos
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
headers = ["a", "b", "s", "t", "z1", "r/2", thetaWheel_headers];

disp('Created Headers')

%Write header + data
fid = fopen('steering.csv', 'w');
fprintf(fid, '%s,', headers{1:end-1});
fprintf(fid, '%s\n', headers{end});
fclose(fid);

%Append the matrix
dlmwrite('steering.csv', output, '-append');

disp('Print to CSV')

disp('done')
