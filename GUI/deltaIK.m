function [t1,t2,t3] = deltaIK(x0,y0,z0)
cos120 = cos(120*pi/180);
sin120 = sin(120*pi/180);

t1 = delta_calcAngleYZ(x0, y0, z0);
t2 = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0); 
t3 = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0); 

% fprintf('t1 = %.2f, t2 = %.2f, t3 = %.2f\n',t1,t2,t3)

end