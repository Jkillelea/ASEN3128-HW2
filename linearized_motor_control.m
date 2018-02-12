function delta_motors = linearized_motor_control(r_dist, k, state)
  % Control moments are 0.004*rotation_rate in their respoective directions
  gain = 0.004;
  K1 = 1.36e-3;
  K2 = 10.625e-3;
  % TODO: These two - derive!
  K3 = 0.00184;
  K4 = 0.014375;

  deltaVel = state(4:6);
  deltaUE  = deltaVel(1);
  deltaVE  = deltaVel(2);
  deltaWE  = deltaVel(3);

  deltaPose  = state(7:9);
  deltaPhi   = deltaPose(1);
  deltaTheta = deltaPose(2);
  deltaPsi   = deltaPose(3);

  deltaOmega = state(10:12);
  deltaP = deltaOmega(1);
  deltaQ = deltaOmega(2);
  deltaR = deltaOmega(3);

  deltaLc = -K1 * deltaP -K2 * deltaPhi;
  deltaMc = -K3 * deltaQ -K4 * deltaTheta;

  matrix = [(r_dist)*[-1  1  1 -1];
            (r_dist)*[-1 -1  1  1];
                 (k)*[-1  1 -1  1];
                     [-1 -1 -1 -1]];

  torqe_command = [ deltaLc;
                    deltaMc;
                    -gain*deltaR;
                    0 ]; % desired Z control force - zero.
                         % note that the control force that counters weight
                         % was accounted for in the original linearization
                         % of the equation set
  delta_motors = matrix\torqe_command;
end
