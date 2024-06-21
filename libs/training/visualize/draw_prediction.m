function draw_prediction(states_vehicles, dt, figure_dir)

    figure;
    hold on;
    N = 10;
    offset = 5;
    
    for i = 1:2:20
        hp = plot(dt(:,2*offset+i), dt(:,2*offset+i+1), 'LineStyle', ':', 'LineWidth', 1.8, 'Color', [0.5, 0.5, 0.5]);
    end
    
    ev = states_vehicles{1}(offset+1:offset+11, :);
    tv = states_vehicles{2}(offset+1:offset+11, :);
    tv_hist = states_vehicles{2}(1:offset+1, :);
    
    lane_width = 5.25;
    
    plot(20+[0, 100], [0, 0], 'LineWidth', 3, 'Color', [0, 0, 0]/256);
    plot(20+[0, 100], [1, 1]*5.25, 'LineStyle', '-.', 'LineWidth', 1.5, 'Color', [0, 0, 0]/256);
    plot(20+[0, 100], [2, 2]*5.25, 'LineStyle', '-.', 'LineWidth', 1.5, 'Color', [0, 0, 0]/256);
    plot(20+[0, 100], [3, 3]*5.25, 'LineWidth', 3, 'Color', [0, 0, 0]/256);
    he = plot(ev(:, 1), ev(:, 2), 'LineWidth', 1.5, 'Color', [51, 51, 255]/256);
    ht = plot(tv(:, 1), tv(:, 2), 'LineWidth', 1.5, 'Color', [255, 26, 26]/256);
    hh = plot(tv_hist(:, 1), tv_hist(:, 2), 'LineWidth', 1.5, 'Color', [26, 26, 26]/256);
    
    for i = [1,4,7,11]
        plot(ev(i, 1), ev(i, 2), 's', 'LineWidth', 1.5, 'Color', [151-10*i, 151-10*i, 255]/256, 'MarkerFaceColor', [151-10*i, 151-10*i, 255]/256);
        plot(tv(i, 1), tv(i, 2), 's', 'LineWidth', 1.5, 'Color', [255, 131-10*i, 131-10*i]/256, 'MarkerFaceColor', [255, 131-10*i, 131-10*i]/256);
    end
    
    % plot(ev(end, 1), ev(end, 2), 's', 'LineWidth', 1.5, 'Color', [51, 51, 255]/256, 'MarkerFaceColor', [51, 51, 255]/256);
    % plot(tv(end, 1), tv(end, 2), 's', 'LineWidth', 1.5, 'Color', [255, 30, 30]/256, 'MarkerFaceColor', [255, 30, 30]/256);
    % 
    % plot(ev(6, 1), ev(5, 2), 's', 'LineWidth', 1.5, 'Color', [100, 100, 255]/256, 'MarkerFaceColor', [100, 100, 255]/256);
    % plot(tv(5, 1), tv(5, 2), 's', 'LineWidth', 1.5, 'Color', [255, 90, 90]/256, 'MarkerFaceColor', [255, 90, 90]/256);
    % 
    % plot(ev(1, 1), ev(1, 2), 's', 'LineWidth', 1.5, 'Color', [150, 150, 255]/256, 'MarkerFaceColor', [150, 150, 255]/256);
    % plot(tv(1, 1), tv(1, 2), 's', 'LineWidth', 1.5, 'Color', [255, 150, 150]/256, 'MarkerFaceColor', [255, 150, 150]/256);
    
    
    
    axis([16, 124, -1, 16]);
    
    title(sprintf('Trajectory prediction'),'interpreter','latex','FontSize',12)
    ylabel('$y$ (m)','Interpreter','latex', 'FontSize', 12);
    xlabel('$x$ (m)','Interpreter','latex', 'FontSize', 12);   
    
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    ax.XAxis.TickLabelInterpreter = 'latex';

    x0=500;
    y0=50;
    width=500;
    height=180;
    
    set(gcf,'position',[x0,y0,width,height]);
    legend([he, ht, hp], {'EV','TV', 'Prediction'}, 'Orientation', 'horizontal');
    hold off;
    saveas(gcf, fullfile(figure_dir, 'segment.svg'));