function draw_training_rmse(training_rmse)

    figure;
    plot(training_rmse);
    title('Training RMSE', 'FontSize', 12);
    ylabel('RMSE','Interpreter','latex', 'FontSize', 12);
    xlabel('iterations','Interpreter','latex', 'FontSize', 12);
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    ax.XAxis.TickLabelInterpreter = 'latex';
    grid on;