figure;
scatter(logger.angleindex/1000*pi, logger.intercepting, 'filled');
set(gca, 'DataAspectRatio',[100 100 10]);
title(['Pure Proportional Navigation, N = ' num2str(N) ])
xlabel('Pursuer Firing Angle (radian)')
ylabel('Interception Indicator')