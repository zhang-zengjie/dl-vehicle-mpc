function out = calculate_cell_rmse(cell_A, cell_B)
    
    num = max(size(cell_A));
    out = zeros(1, num);
    for i = 1:num
        out(i) = rmse(cell_A{i}, cell_B{i}, "all");
    end