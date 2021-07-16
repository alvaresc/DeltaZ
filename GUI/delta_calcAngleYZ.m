function theta = delta_calcAngleYZ(x0,y0,z0)
e = 25;
f = 50;
re = 60.0;
rf = 30.0;
y1 = -0.5 * 0.57735 * f;
y0 = y0-(0.5 * 0.57735    * e);
a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
b = (y1 - y0) / z0;
d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
if d < 0
    theta = -1;
end
yj = (y1 - a * b - sqrt(d)) / (b * b + 1);
zj = a + b * yj;

if yj > y1
    theta = 180.0 * atan(-zj / (y1 - yj)) / pi +  180.0;
else
    theta = 180.0 * atan(-zj / (y1 - yj)) / pi;
end
end