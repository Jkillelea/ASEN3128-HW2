function Q = Q_yaw_pitch_roll(psi, theta, phi)
  % psi   = vec(1); % euler yaw
  % theta = vec(2); % euler pitch
  % phi   = vec(3); % euler roll

  Q = R1(phi)*R2(theta)*R3(psi);
end
