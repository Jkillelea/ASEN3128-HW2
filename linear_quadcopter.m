function results = linear_quadcopter(t, y, opts)
  m = 0.068;     % kg
  g = 9.81;      % m/s^2
  k = 0.0024;    % m
  d = 0.06;      % m
  r_dist = d/sqrt(2); % lateral distance from fan to CM, m

  Ix = 6.8e-5; % moments of inertia about the body axes
  Iy = 9.2e-5;
  Iz = 1.35e-4;

  % OUTPUTS
  % deltaPDot, deltaQDot, deltaRDot
  % deltaUEdot, deltaVEdot, deltaWEdot
  % deltaThetaDot, deltaPhiDot, deltaPsiDot

  % PARAMS
  % opts -> currently unused
  % deltaF1, deltaF2, deltaF3, deltaF4
  % deltaTheta, deltaPhi
  % deltaQ, deltaP, deltaR

  % deltaPos   = y(1:3); % unused
  deltaPosDot = [0; 0; 0];

  % just hardcoding the motors for now
  deltaF1 = 0;
  deltaF2 = 0;
  deltaF3 = 0;
  deltaF4 = 0;

  % velocity
  deltaVel   = y(4:6);
  deltaUEdot = deltaVel(1);
  deltaVEdot = deltaVel(2);
  deltaWEdot = deltaVel(3);

  % euler angle pose
  deltaPose  = y(7:9);
  deltaPhi   = deltaPose(1);
  deltaTheta = deltaPose(2);
  deltaPsi   = deltaPose(3);

  % body angular rates
  deltaOmega = y(10:12);
  deltaP = deltaOmega(1);
  deltaQ = deltaOmega(2);
  deltaR = deltaOmega(3);

  % I'm not being consistent with my camelCase since I want the names to all look distinct.
  deltaUEdot = -g * deltaTheta;
  deltaVEdot =  g * deltaPhi;
  deltaWEdot = -(1/m) * (deltaF1 + deltaF2 + deltaF3 + deltaF4);

  deltaVelDot = [deltaUEdot; deltaVEdot; deltaWEdot];

  deltaPhiDot   = deltaP;
  deltaThetaDot = deltaQ;
  deltaPsiDot   = 0; % not in any equation

  deltaPoseDot = [deltaPhiDot; deltaThetaDot; deltaPsiDot];

  deltaPDot = (r_dist/Ix)*(-deltaF1 + deltaF2 + deltaF3 - deltaF4);
  deltaQDot = (r_dist/Iy)*(-deltaF1 - deltaF2 + deltaF3 + deltaF4);
  deltaRDot =      (k/Iz)*(-deltaF1 + deltaF2 - deltaF3 + deltaF4);

  deltaOmegaDot = [deltaPDot; deltaQDot; deltaRDot];

  results = [deltaPosDot; deltaVelDot; deltaPoseDot; deltaOmegaDot];
end
