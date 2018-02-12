function delta_motors = linearized_motor_control(r_dist, k, Ix, Iy, Iz, state)
  % Control moments are 0.004*rotation_rate in their respoective directions
  yawGain = 0.004;

  tau  = 0.1;
  zeta = 0.8;
  wn   = 1/(tau*zeta);

  K1 = 2*zeta*wn*Ix;
  K2 = (wn^2)*Ix;

  K3 = 2*zeta*wn*Iy;
  K4 = (wn^2)*Iy;

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
                   -yawGain*deltaR;
                    0 ]; % desired Z control force - zero.
                         % note that the control force that counters weight
                         % was accounted for in the original linearization
                         % of the equation set
  delta_motors = matrix\torqe_command;
end
