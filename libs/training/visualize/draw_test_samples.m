function draw_test_samples(sample_traj, label_traj, prediction_traj)
    
    figure;
    hold on;
    plot(sample_traj(1,:), sample_traj(2,:), 'Color', [0, 0, 1]);
    plot(label_traj(1,:), label_traj(2,:), 'Color', [0, 1, 0])
    plot(prediction_traj(1,:), prediction_traj(2,:), 'Color', [1, 0, 0])
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    ax.XAxis.TickLabelInterpreter = 'latex';
    grid on;
    hold off;
    % axis([-25, 55, -20, 30]);