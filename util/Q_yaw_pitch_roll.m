function Q = Q_yaw_pitch_roll(vec)
  psi   = vec(1); % euler yaw
  theta = vec(2); % euler pitch
  phi   = vec(3); % euler roll
  Q = R3(psi)*R2(theta)*R1(phi);
end
