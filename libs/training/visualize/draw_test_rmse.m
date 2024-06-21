function draw_test_rmse(RMSE_vec)

    figure;
    histogram(RMSE_vec);
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    ax.XAxis.TickLabelInterpreter = 'latex';
    grid on;