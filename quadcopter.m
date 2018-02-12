% quadcopter physics simulation
function results = quadcopter(t, y, opts)
  m = 0.068;     % kg
  g = 9.81;      % m/s^2
  k = 0.0024;    % m
  d = 0.06;      % m
  r_dist = d/sqrt(2); % lateral distance from fan to CM, m

  Ix = 6.8e-5; % moments of inertia about the body axes
  Iy = 9.2e-5;
  Iz = 1.35e-4;

  % Lateral and Vertical coefficients of drag
  CdLat  = 1e-3; % I don't know what the fancy greek symbol for this is.
  CdVert = 3e-3;

  % aerodynamic Moment coefficients
  alpha = 2e-6;
  beta  = 1e-6;

  % r     = y(1:3);   % inertial position
  vel   = y(4:6);   % body velocity
  pose  = y(7:9);   % roll, pitch, yaw (euler angles)
  omega = y(10:12); % body angular velocity

  aero  = opts.aero; % aerodynamic drag and moments
  control_motor = opts.control_motor; % whether motors just balance weight

  u     = vel(1);   % body x velocity
  v     = vel(2);   % body y velocity
  w     = vel(3);   % body z velocity
  phi   = pose(1);  % roll  (euler angle)
  theta = pose(2);  % pitch (euler angle)
  psi   = pose(3);  % yaw   (euler angle)
  p     = omega(1); % roll  rate
  q     = omega(2); % pitch rate
  r     = omega(3); % yaw   rate

  if control_motor
    [f1, f2, f3, f4] = motor_control(m, g, Ix, Iy, Iz, y);
  else
    f1 = m*g/4;
    f2 = m*g/4;
    f3 = m*g/4;
    f4 = m*g/4;
  end

  Qeb = R3(-psi)*R2(-theta)*R1(-phi); % transform body vector to inertial vector
  Qbe = R3(psi)*R2(theta)*R1(phi);    % transform inertial vector to body vector

  dr = Qeb * vel; % inertial velocity is the change in position vector

  control_force = [0; 0; (-f1 - f2 - f3 - f4)];

  % aerodynamic drag
  if aero
    air_force = [ -CdLat*(u^2)*sign(u);
                  -CdLat*(v^2)*sign(v);
                  -CdVert*(w^2)*sign(w) ];
  else
    air_force = [0; 0; 0];
  end

  aerodynamic_forces = control_force + air_force;
  X = aerodynamic_forces(1);
  Y = aerodynamic_forces(2);
  Z = aerodynamic_forces(3);

  % equations (4.7, 1)
  udot_E = X/m - g*sin(theta)          + r*v - q*w;
  vdot_E = Y/m + g*cos(theta)*sin(phi) + p*w - r*u;
  wdot_E = Z/m + g*cos(theta)*cos(phi) + q*u - p*v;

  dvel = [udot_E; vdot_E; wdot_E]; % change in u, v, and w

  control_moment = [ r_dist*( f2 + f3 - f1 - f4);
                     r_dist*(-f1 - f2 + f3 + f4);
                          k*( f2 + f4 - f1 - f3) ];

  % aerodynamic moment
  if aero
    aero_moment = [ -alpha*(p^2)*sign(p);
                    -alpha*(q^2)*sign(q);
                     -beta*(r^2)*sign(r) ];
  else
    aero_moment = [0; 0; 0];
  end

  moments = control_moment + aero_moment;
  L = moments(1);
  M = moments(2);
  N = moments(3);

  % equations (4.7, 2)
  pdot = (L - q*r*(Iz - Iy))/Ix; % angular accelerations in the body frame
  qdot = (M - r*p*(Ix - Iz))/Iy;
  rdot = (N - p*q*(Iy - Ix))/Iz;

  domega = [pdot; qdot; rdot]; % change in p, q, and r

  % equations (4.7, 3)
  dphi   = p + (q*sin(phi) + r*cos(phi))*tan(theta);
  dtheta = q*cos(phi) - r*sin(phi);
  dpsi   = (q*sin(phi) + r*cos(phi))*sec(theta);

  dpose = [dphi; dtheta; dpsi]; % change in phi, theta, and psi

  results = [dr; dvel; dpose; domega];
end
