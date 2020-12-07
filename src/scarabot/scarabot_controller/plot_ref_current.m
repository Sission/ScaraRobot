M1 = table2array(readtable("data.csv"));
M2 = table2array(readtable("data2.csv"));
M3 = table2array(readtable("data3.csv"));
figure(1)
plot_position(M1);
figure(2)
plot_position(M2);
figure(3)
plot_position(M3);

function plot_position(M)
    plot(M(:,1)-M(1,1),M(:,2));
    hold on;
    plot(M(:,1)-M(1,1),M(:,3));
    xlim([0 M(end,1)-M(1,1)]);
    legend('current position','reference position');
    xlabel('time');
    ylabel('position');
    hold off;
end