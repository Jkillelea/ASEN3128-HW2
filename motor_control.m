function [f1, f2, f3, f4] = motor_control(m, g, state)
  d = 0.06;      % m
  r_dist = d/sqrt(2); % lateral distance from fan to CM, m
  k = 0.0024;
  gain = 0.004;

  omega = state(10:12); % body angular velocity
  p = omega(1);
  q = omega(2);
  r = omega(3);

  f1 = m*g/4;
  f2 = m*g/4;
  f3 = m*g/4;
  f4 = m*g/4;

  % Control moments are 0.004*rotation_rate in their respoective directions
  gain = 0.004;

  matrix = [(r_dist)*[-1  1  1 -1];
            (r_dist)*[-1 -1  1  1];
                 (k)*[-1  1 -1  1];
                     [-1 -1 -1 -1]];

  torqe_command = -gain*[ p;
                          q;
                          r;
                          0 ]; % desired Z control force - zero.
                               % note that the control force that counters weight
                               % was accounted for in the original linearization
                               % of the equation set
  delta_motors = matrix\torqe_command;

  f1 = f1 + delta_motors(1);
  f2 = f2 + delta_motors(2);
  f3 = f3 + delta_motors(3);
  f4 = f4 + delta_motors(4);
end
