% quadcopter physics simulation
function results = quadcopter(t, y)

  m  = 0.068; % kg
  g  = 9.81;  % m/s^2
  d  = 0.06;  % m

  Ix = 6.8e-5; % moments of inertia about the body axes
  Iy = 9.2e-5;
  Iz = 1.35e-4;

  x     = y(1:3);   % position
  dx    = y(4:6);   % velocity
  pose  = y(7:9);   % roll pitch yaw
  dpose = y(10:12); % angular velocity (inertial)

  phi   = pose(1);  % roll
  theta = pose(2);  % pitch
  psi   = pose(3);  % yaw

  Q = R1(phi)*R2(theta)*R3(psi); % direction cosine matrix

  % CHECK this
  vel = Q * dx; % body frame translational velocity
  u = vel(1);
  v = vel(2);
  w = vel(3);

  omega = Q * dpose; % body frame angular velocity
  p = omega(1);
  q = omega(2);
  r = omega(3);

  % TODO -> define these guys
  control_force = [0; 0; 0];
  air_force     = [0; 0; 0];

  aerodynamic_forces = control_force + air_force;
  X = aerodynamic_forces(1);
  Y = aerodynamic_forces(2);
  Z = aerodynamic_forces(3);

  % equations (4.7, 1)
  udot_E = X/m - g*sin(theta)          + r*v - q*w;
  vdot_E = Y/m + g*cos(theta)*sin(phi) + p*w - r*u;
  wdot_E = Z/m + g*cos(theta)*cos(phi) + q*u - p*v;

  ddx = (Q^-1) * [udot_E; vdot_E; wdot_E]; % inertial acceleration

  % equations (4.7, 4)
  xdot_E = u*cos(theta)*cos(psi)                                             ...
         + v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))              ...
         + w*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
  ydot_E = u*cos(psi)                                                        ...
         + v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))              ...
         + w*(cos(phi)*sin(theta)*sin(psi) - sin(theta)*cos(psi));
  zdot_E = -u*sin(theta)                                                     ...
         + v*sin(phi)*cos(theta)                                             ...
         + w*cos(phi)*cos(theta);

  dx = [xdot_E; ydot_E; zdot_E]; % inertial velocity

  % results = [dx; ddx; dpose; ddpose];
  results = zeros(12, 1);
end
