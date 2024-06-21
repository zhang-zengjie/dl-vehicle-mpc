function draw_test_rmse(RMSE_vec, figure_dir)

    figure;
    histogram(RMSE_vec, 100);
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    ax.XAxis.TickLabelInterpreter = 'latex';
    title(sprintf('Distribution of RMSE in test'),'interpreter','latex','FontSize',12)
    ylabel('RMSE','Interpreter','latex', 'FontSize', 12);
    xlabel('Number of tests','Interpreter','latex', 'FontSize', 12); 

    x0=500;
    y0=50;
    width=500;
    height=180;
    
    set(gcf,'position',[x0,y0,width,height]);
    grid on;
    saveas(gcf, fullfile(figure_dir, 'rmse_test.svg'));