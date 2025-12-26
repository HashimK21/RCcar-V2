clc

thetak = 9.5;
KPL = -8.6;
tireW = 34; %tire width
tw = 220; %track width
xj = (tw/2) - (tireW/2);


y1 = 20.5;
deltay = 20;
y2 = y1 + deltay;



c = deltay .* tand(thetak);
KPU = KPL + c;
u = xj - KPU;
l = xj - KPL;
x = ((-y2 .* (u - l)) ./ deltay) + u;
rs = 110 - x;

k = (KPU + KPL)./ 2

alphac = 6;
zc = deltay .* tand(alphac);






