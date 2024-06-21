function draw_test_samples(sample_traj, label_traj, prediction_traj, figure_dir)
    
    figure;
    hold on;
    he = plot(sample_traj(1,:), sample_traj(2,:), 'LineWidth', 3, 'Color', [0, 0, 1]);
    ht = plot(label_traj(1,:), label_traj(2,:), 'LineWidth', 3, 'Color', [0, 1, 0]);
    hp = plot(prediction_traj(1,:), prediction_traj(2,:), '-.', 'LineWidth', 3, 'Color', [1, 0, 0]);
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    ax.XAxis.TickLabelInterpreter = 'latex';

    title(sprintf('Prediction example'),'interpreter','latex','FontSize',12)

    ylabel('$y$ (m)','interpreter','latex','FontSize', 12)
    xlabel('$x$ (m)','interpreter','latex','FontSize', 12)
    grid on;
    hold off;
    axis([-100, 100, -4, 4]);
    legend([he, ht, hp], {'History','Ground truth', 'Prediction'}, 'Location', 'southeast', 'Orientation', 'vertical');

    x0=500;
    y0=50;
    width=500;
    height=180;
    
    set(gcf,'position',[x0,y0,width,height]);
    saveas(gcf, fullfile(figure_dir, 'prediction.svg'));