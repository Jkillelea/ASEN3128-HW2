function results = linear_quadcopter(t, y, opts, linearization_origin)
  % Linearized quadcopter model
  % reports deltas from the hover state (linearization_origin)

  % OUTPUTS
  % deltaPDot, deltaQDot, deltaRDot
  % deltaUEdot, deltaVEdot, deltaWEdot
  % deltaThetaDot, deltaPhiDot, deltaPsiDot

  % PARAMS
  % opts -> options struct
  % linearization_origin -> conditions we've linearized about
  % deltaF1, deltaF2, deltaF3, deltaF4
  % deltaTheta, deltaPhi
  % deltaQ, deltaP, deltaR
  control_motor = opts.control_motor;

  m = 0.068;     % kg
  g = 9.81;      % m/s^2
  k = 0.0024;    % m
  d = 0.06;      % m
  r_dist = d/sqrt(2); % lateral distance from fan to CM, m

  Ix = 6.8e-5; % moments of inertia about the body axes
  Iy = 9.2e-5;
  Iz = 1.35e-4;

  % origin points
  pos0   = linearization_origin(1:3);
  vel0   = linearization_origin(1:3);
  pose0  = linearization_origin(1:3);
  omega0 = linearization_origin(1:3);
  phi0   = pose0(1);
  theta0 = pose0(2);
  psi0   = pose0(3);

  deltaPos   = y(1:3); % unused

  % velocity
  deltaVel = y(4:6);
  deltaUE  = deltaVel(1);
  deltaVE  = deltaVel(2);
  deltaWE  = deltaVel(3);

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

  % just hardcoding the motors for now
  if control_motor
    deltaMotors = linearized_motor_control(r_dist, k, y);
    deltaF1 = deltaMotors(1);
    deltaF2 = deltaMotors(2);
    deltaF3 = deltaMotors(3);
    deltaF4 = deltaMotors(4);
  else
    deltaF1 = 0;
    deltaF2 = 0;
    deltaF3 = 0;
    deltaF4 = 0;
  end


  phi   = deltaPhi   + phi0;
  theta = deltaTheta + theta0;
  psi   = deltaPsi   + psi0;
  Qeb   = R3(-psi)*R2(-theta)*R1(-phi); % transform body vector to inertial vector
  Qbe   = R3(psi)*R2(theta)*R1(phi);    % transform inertial vector to body vector

  deltaPosDot = Qeb * deltaVel; % change in position

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
