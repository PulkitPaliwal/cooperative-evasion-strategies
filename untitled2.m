plot(logger.weight, logger.min_vel); hold on;
set(gca, 'DataAspectRatio',[1 0.5 1]);
title(['Pure Proportional Navigation, N = ' num2str(N) ])
legend('weight', 'min defender launch', 'Orientation','horizontal')
xlabel('+E (m)')
ylabel('+N (m)')