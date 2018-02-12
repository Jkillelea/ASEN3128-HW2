function [f1, f2, f3, f4] = motor_control(m, g, Ix, Iy, Iz, state)
  d      = 0.06;      % m
  r_dist = d/sqrt(2); % lateral distance from fan to CM, m
  k      = 0.0024;
  yawGain = 0.004;

  tau  = 0.1;
  zeta = 0.8;
  wn   = 1/(tau*zeta);

  % roll gains
  K1 = 2*zeta*wn*Ix; % derivative
  K2 = (wn^2)*Ix;    % porportional
  % pitch gains
  K3 = 2*zeta*wn*Iy; % derivative
  K4 = (wn^2)*Iy;    % porportional

  pose  = state(7:9);
  phi   = pose(1);
  theta = pose(2);
  psi   = pose(3);

  omega = state(10:12);
  p = omega(1);
  q = omega(2);
  r = omega(3);

  Lc = -K1 * p -K2 * phi;
  Mc = -K3 * q -K4 * theta;

  matrix = [(r_dist)*[-1  1  1 -1];
            (r_dist)*[-1 -1  1  1];
                 (k)*[-1  1 -1  1];
                     [-1 -1 -1 -1]];

 torqe_command = [ Lc;
                   Mc;
                  -yawGain*r;
                   0 ]; % desired Z control force - zero.
                        % note that the control force that counters weight
                        % was accounted for in the original linearization
                        % of the equation set
  delta_motors = matrix\torqe_command;

  f1 = m*g/4; % weight
  f2 = m*g/4;
  f3 = m*g/4;
  f4 = m*g/4;
  f1 = f1 + delta_motors(1); % add control force
  f2 = f2 + delta_motors(2);
  f3 = f3 + delta_motors(3);
  f4 = f4 + delta_motors(4);
end
