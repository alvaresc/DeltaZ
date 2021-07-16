function inWorkspace = forwardKinTest( theta1,  theta2,  theta3, R, ZMin, ZMax)
  e = 25;      
  f = 50;      
  re = 60.0;
  rf = 30.0;
  sqrt3 = sqrt(3.0);
  sin120 = sqrt3 / 2.0;
  cos120 = -0.5;
  tan60 = sqrt3;
  sin30 = 0.5;
  tan30 = 1.0 / sqrt3;
  t = (f - e) * tan30 / 2;

  theta1 = theta1 * pi/180;
  theta2 = theta2 * pi/180;
  theta3 = theta3 * pi/180;

  y1 = -(t + rf * cos(theta1));
  z1 = -rf * sin(theta1);

  y2 = (t + rf * cos(theta2)) * sin30;
  x2 = y2 * tan60;
  z2 = -rf * sin(theta2);

  y3 = (t + rf * cos(theta3)) * sin30;
  x3 = -y3 * tan60;
  z3 = -rf * sin(theta3);

  dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  w1 = y1 * y1 + z1 * z1;
  w2 = x2 * x2 + y2 * y2 + z2 * z2;
  w3 = x3 * x3 + y3 * y3 + z3 * z3;

  a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  a = a1 * a1 + a2 * a2 + dnm * dnm;
  b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

  d = b * b - 4.0 * a * c;
  if d < 0
      inWorkspace = 0;      
  else
      z0 = -0.5 * (b + sqrt(d)) / a;
      x0 = (a1 * z0 + b1) / dnm;
      y0 = (a2 * z0 + b2) / dnm;
      if sqrt(x0*x0+y0*y0) <= R && z0>=ZMin && z0<=ZMax
          inWorkspace = 1;
      else
           inWorkspace = 0;
      end
      
  end

end

