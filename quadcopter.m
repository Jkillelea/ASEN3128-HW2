% quadcopter physics simulation
function results = quadcopter(t, y)

  m = 0.068; % kg
  d = 0.06;  % m

  Ii = 6.8e-5; % moments of inertia about the body axes
  Ij = 9.2e-5;
  Ik = 1.35e-4;

  x     = y(1:3);   % position
  dx    = y(4:6);   % velocity
  pose  = y(7:9);   % roll pitch yaw
  dpose = y(10:12); % angular velocity

  phi   = pose(1);  % roll
  theta = pose(2);  % pitch
  psi   = pose(3);  % yaw

  Q = R1(phi)*R2(theta)*R3(psi); % direction cosine matrix

  vel = Q * dx;

  rel_wind = -vel; % no wind condition

  % dx = (Q^-1) * vel;
  % forces = zeros(3, 1);
  % ddx = forces/m;
  % dpose = [0; 0; 0];
  % ddpose = [0; 0; 0];

  results = zeros(12, 1);
  % results = [dx; ddx; dpose; ddpose];
end
