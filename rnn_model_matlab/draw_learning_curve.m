RMSE = [49.56;
        30.39;
        26.21;
        20.78;
        19.76;
        18.43;
        15.99;
        17.26;
        15.06;
        15.96;
        15.42;
        15.13;
        15.33;
        15.11;
        15.11;
        15.84;
        14.99;
        16.03;
        15.92];

ITER = [   1;
          50;
         100;
         150;
         200;
         250;
         300;
         350;
         400;
         450;
         500;
         550;
         600;
         650;
         700;
         750;
         800;
         850;
         900];

bar(ITER, RMSE);

ylabel('RMSE','Interpreter','latex', 'FontSize', 11);
grid on;
xlabel('iterations','Interpreter','latex', 'FontSize', 11);   

x0=500;
y0=50;
width=500;
height=180;

set(gcf,'position',[x0,y0,width,height]);