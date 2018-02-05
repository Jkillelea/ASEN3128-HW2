function [f1, f2, f3, f4] = motor_control(m, g, state)
  d = 0.06;      % m
  r = d/sqrt(2); % lateral distance from fan to CM, m
  k = 0.0024;

  force = 0.6666128353; % N
  f1 = force/4;
  f2 = force/4;
  f3 = force/4;
  f4 = force/4;
end
