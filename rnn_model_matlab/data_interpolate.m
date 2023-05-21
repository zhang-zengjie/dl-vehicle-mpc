function traj_out = data_interpolate(x, yf, y0)

    a_0 = y0;
    a_1 = 0;
    a_2 = 3*(yf - y0) / (x(end)^2);
    a_3 = -2*(yf - y0) / (x(end)^3);
    traj_out = a_0 + a_1 * x + a_2 * (x.^2) + a_3*(x.^3);