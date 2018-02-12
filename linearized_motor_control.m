function delta_motors = linearized_motor_control(deltaP, deltaQ, deltaR, r_dist, k)
  % Control moments are 0.004*rotation_rate in their respoective directions
  gain = 0.004;

  matrix = [(r_dist)*[-1  1  1 -1];
            (r_dist)*[-1 -1  1  1];
                 (k)*[-1  1 -1  1];
                     [-1 -1 -1 -1]];

  torqe_command = -gain*[ deltaP;
                          deltaQ;
                          deltaR;
                          0 ]; % desired Z control force - zero.
                               % note that the control force that counters weight
                               % was accounted for in the original linearization
                               % of the equation set
  delta_motors = matrix\torqe_command;
end
