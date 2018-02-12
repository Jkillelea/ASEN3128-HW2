function plotQuadCopterDynamics(t, data)
  r     = data(:, 1:3);
  v     = data(:, 4:6);
  pose  = data(:, 7:9);
  omega = data(:, 10:12);

  figure('units', 'normalized', 'outerposition', [0 0 1 1]); % fullscreen
  hold on; grid on;

  subplot(2, 2, 1); hold on;
  plot(t, r(:, 1), 'DisplayName', 'N', 'LineWidth', 2)
  plot(t, r(:, 2), 'DisplayName', 'E', 'LineWidth', 2)
  plot(t, r(:, 3), 'DisplayName', 'D', 'LineWidth', 2)
  xlabel('Time (sec)');
  ylabel('Displacement (m)');
  legend('show', 'location', 'southwest')

  subplot(2, 2, 2); hold on;
  plot(t, v(:, 1), 'DisplayName', 'u', 'LineWidth', 2) % body x velocity
  plot(t, v(:, 2), 'DisplayName', 'v', 'LineWidth', 2) % y
  plot(t, v(:, 3), 'DisplayName', 'w', 'LineWidth', 2) % z
  xlabel('Time (sec)');
  ylabel('Velocity (m/s)');
  legend('show', 'location', 'southwest')

  subplot(2, 2, 3); hold on;
  plot(t, pose(:, 1), 'DisplayName', '\Phi',   'LineWidth', 2)
  plot(t, pose(:, 2), 'DisplayName', '\Theta', 'LineWidth', 2)
  plot(t, pose(:, 3), 'DisplayName', '\Psi',   'LineWidth', 2)
  xlabel('Time (sec)');
  ylabel('Euler Angles');
  legend('show', 'location', 'southwest')

  subplot(2, 2, 4); hold on;
  plot(t, omega(:, 1), 'DisplayName', 'p', 'LineWidth', 2)
  plot(t, omega(:, 2), 'DisplayName', 'q', 'LineWidth', 2)
  plot(t, omega(:, 3), 'DisplayName', 'r', 'LineWidth', 2)
  xlabel('Time (sec)');
  ylabel('Body Angular Velocity (rad/s)');
  legend('show', 'location', 'southwest')
end
