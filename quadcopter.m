% quadcopter physics simulation
function results = quadcopter(t, y)

  m  = 0.068; % kg
  g  = 9.81;  % m/s^2
  d  = 0.06;  % m

  Ix = 6.8e-5; % moments of inertia about the body axes
  Iy = 9.2e-5;
  Iz = 1.35e-4;

  r     = y(1:3);   % inertial position
  vel   = y(4:6);   % body velocity
  pose  = y(7:9);   % roll, pitch, yaw (inertial)
  omega = y(10:12); % body angular velocity

  u      = vel(1);   % body x velocity
  v      = vel(2);   % body y velocity
  w      = vel(3);   % body z velocity
  phi    = pose(1);  % roll  (inertial)
  theta  = pose(2);  % pitch (inertial)
  psi    = pose(3);  % yaw   (inertial)
  p      = omega(1); % roll  rate
  q      = omega(2); % pitch rate
  r      = omega(3); % yaw   rate

  % HAHAHAHAH WHATS OUR VECTOR VICTOR HAHAHAHAHA
  Qeb = R3(-psi)*R2(-theta)*R1(-phi); % transform body vector to inertial vector
  Qbe = R3(psi)*R2(theta)*R1(phi);    % transform inertial vector to body vector

  dr = Qeb * vel; % inertial velocity is the change in position vector

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

  dvel = [udot_E; vdot_E; wdot_E];

  % TODO
  aero_moment = [0; 0; 0];
  L = aero_moment(1);
  M = aero_moment(2);
  N = aero_moment(3);

  % equations (4.7, 2)
  pdot = (L - q*r*(Iz - Iy))/Ix; % angular accelerations in the body frame
  qdot = (M - r*p*(Ix - Iz))/Iy;
  rdot = (N - p*q*(Iy - Ix))/Iz;

  domega = [pdot; qdot; rdot];

  % equations (4.7, 3)
  dphi   = p + (q*sin(phi) + r*cos(phi))*tan(theta);
  dtheta = q*cos(phi) - r*sin(phi);
  dpsi   = (q*sin(phi) + r*cos(phi))*sec(theta);

  dpose = [dphi; dtheta; dpsi];


  results = [dr; dvel; dpose; domega];


  % results = zeros(12, 1);
end
