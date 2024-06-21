function draw_training_rmse(training_rmse, figure_dir)

    figure;
    plot(training_rmse);
    title(sprintf('RMSE in training'),'interpreter','latex','FontSize',12)
    ylabel('RMSE','Interpreter','latex', 'FontSize', 12);
    xlabel('iterations','Interpreter','latex', 'FontSize', 12);
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    ax.XAxis.TickLabelInterpreter = 'latex';
    axis([0, length(training_rmse), 0, max(training_rmse)]);

    x0=500;
    y0=50;
    width=500;
    height=180;
    
    set(gcf,'position',[x0,y0,width,height]);
    grid on;
    saveas(gcf, fullfile(figure_dir, 'rmse_training.svg'));