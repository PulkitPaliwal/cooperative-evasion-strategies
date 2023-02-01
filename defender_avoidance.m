clear; close all
% Pro-Nav gain, unitless (typically set between 3-5)
N = 4;
w = 0.3;

% Pursuer-1 init prams
pursuer1.N   =  100.0; % north pos, m
pursuer1.E   =  0; % east pos, m
pursuer1.psi =  0; % heading, rad
pursuer1.V   =  1.5; % velocity magnitude, m/s
pursuer1.Nv  =  pursuer1.V * cos(pursuer1.psi); % north velocity, m/s
pursuer1.Ev  =  pursuer1.V * sin(pursuer1.psi); % east velocity, m/s

% Pursuer-2 init prams
pursuer2.N   =  0; % north pos, m
pursuer2.E   =  100; % east pos, m
pursuer2.psi =  0; % heading, rad
pursuer2.V   =  0.0; % velocity magnitude, m/s
pursuer2.Nv  =  pursuer2.V * cos(pursuer2.psi); % north velocity, m/s
pursuer2.Ev  =  pursuer2.V * sin(pursuer2.psi); % east velocity, m/s

% Defender init prams
defender.N   =  250.0; % north pos, m
defender.E   =  250.0; % east pos, m
defender.psi =  pi; % heading, rad
defender.V   =  1; % velocity magnitude, m/s
defender.Nv  =  defender.V * cos(defender.psi); % north velocity, m/s
defender.Ev  =  defender.V * sin(defender.psi); % east velocity, m/s

% Target init prams
target.anchored = false; % fix target to init pos when true
target.N   =  250; % north pos, m
target.E   =  250;   % east pos, m
target.Nv  =  0.4; % north velocity, m/s
target.Ev  =  0.4; % east velocity, m/s
target.V   = sqrt(target.Nv^2 + target.Ev^2); % velocity magnitude, m/s
% Current prams
% Applied as a disturbance to pursuer's and target's velocity
current.Nv =  0.0; % north velocity, m/s
current.Ev =  0.0; % east velocity, m/s
% Sim params
S  = 50000;    % max sim duration, seconds
dt = 0.4;     % time-step size, seconds
Niter = S/dt; % max iteration num
% Pre-allocate logger
logger.t   = nan(1, Niter);    % elapsed time
logger.pN1  = nan(1, Niter);    % pursuer1 north pos
logger.pE1  = nan(1, Niter);    % pursuer1 east pos
logger.pNv1 = nan(1, Niter);    % pursuer1 north vel
logger.pEv1 = nan(1, Niter);    % pursuer1 east vel
logger.psi1 = nan(1, Niter);    % pursuer1 heading angle
logger.pN2  = nan(1, Niter);    % pursuer2 north pos
logger.pE2  = nan(1, Niter);    % pursuer2 east pos
logger.pNv2 = nan(1, Niter);    % pursuer2 north vel
logger.pEv2 = nan(1, Niter);    % pursuer2 east vel
logger.psi2 = nan(1, Niter);    % pursuer2 heading angle
logger.dN1  = nan(1, Niter);    % defender north pos
logger.dE1  = nan(1, Niter);    % defender east pos
logger.dNv1 = nan(1, Niter);    % defender north vel
logger.dEv1 = nan(1, Niter);    % defender east vel
logger.dsi1 = nan(1, Niter);    % defender heading angle
logger.tN  = nan(1, Niter);    % target north pos
logger.tE  = nan(1, Niter);    % target east pos
logger.tNv = nan(1, Niter);    % target north vel
logger.tEv = nan(1, Niter);    % target east vel
logger.R1   = nan(1, Niter);    % pursuer1/target range
logger.Lambda1 = nan(1, Niter); % pursuer1/target bearing
logger.R2   = nan(1, Niter);    % pursuer2/target range
logger.Lambda2 = nan(1, Niter); % pursuer2/target bearing
logger.R12   = nan(1, Niter);    % mutual/target range
logger.Lambda12 = nan(1, Niter); % mutual/target bearing
%--------------------------------------------------------------------------
% Init sim
%--------------------------------------------------------------------------
RR12_last = [(pursuer2.N - pursuer1.N);... % delta N
        (pursuer2.E - pursuer1.E)];      % delta E
RTP_last1 = [(target.N - pursuer1.N);... % delta N
    (target.E - pursuer1.E)];	       % delta E
RTP_last2 = [(target.N - pursuer2.N);... % delta N
    (target.E - pursuer2.E)];	       % delta E
RDP_last1 = [(defender.N - pursuer1.N);... % delta N
    (defender.E - pursuer1.E)];	       % delta E
RDP_last2 = [(defender.N - pursuer2.N);... % delta N
    (defender.E - pursuer2.E)];	       % delta E
% Target pos
target.N = target.N + target.Nv*dt;
target.E = target.E + target.Ev*dt;

% Defender pos
defender.N = defender.N + defender.Nv*dt;
defender.E = defender.E + defender.Ev*dt;

% Pursuer pos
pursuer1.N = pursuer1.N + pursuer1.Nv*dt;
pursuer1.E = pursuer1.E + pursuer1.Ev*dt;
pursuer2.N = pursuer2.N + pursuer2.Nv*dt;
pursuer2.E = pursuer2.E + pursuer2.Ev*dt;
%--------------------------------------------------------------------------
% Run sim
%--------------------------------------------------------------------------
for k = 1:Niter
    RR12 = [(pursuer2.N - pursuer1.N);... % delta N
        (pursuer2.E - pursuer1.E)];      % delta E
    % Relative position in the inertial frame, m
    RTP1 = [(target.N - pursuer1.N);... % delta N
        (target.E - pursuer1.E)];      % delta E
    RTP2 = [(target.N - pursuer2.N);... % delta N
        (target.E - pursuer2.E)];      % delta E
    RDP1 = [(defender.N - pursuer1.N);... % delta N
        (defender.E - pursuer1.E)];      % delta E
    RDP2 = [(defender.N - pursuer2.N);... % delta N
        (defender.E - pursuer2.E)];      % delta E
    
    % Range to target
    R1 = norm(RTP1);
    R2 = norm(RTP2);
    % Range to defender
    RD1 = norm(RDP1);
    RD2 = norm(RDP2);
    % Distance between the pursuers
    R12 = norm(RR12);
    % Relative velocity in the inertial frame, m/s
    % VTP = [(target.Nv - pursuer.Nv);... % delta N velocity
    %        (target.Ev - pursuer.Ev)];   % delta E velocity
    VTP1 = (RTP1 - RTP_last1) ./ dt;
    VTP2 = (RTP2 - RTP_last2) ./ dt;
    VDP1 = (RDP1 - RDP_last1) ./ dt;
    VDP2 = (RDP2 - RDP_last2) ./ dt;
    
    % Closing velocity, m/s
    % Vc = -RTP'*VTP / R;
    
    % Pursuer velocity, m/s
    Vp1 = sqrt(pursuer1.Nv^2 + pursuer1.Ev^2);
    Vp2 = sqrt(pursuer2.Nv^2 + pursuer2.Ev^2);
    Vd  = sqrt(defender.Nv^2 + defender.Ev^2);
    % Target velocity, m/s
    % Vt = sqrt(target.Nv^2 + target.Ev^2);
    
    % Line-of-sight (LOS) angle, rad
    lambda1 = atan2(RTP1(2), RTP1(1));
    lambda2 = atan2(RTP2(2), RTP2(1));
    lambdaD1 = atan2(RDP1(2), RDP1(1));
    lambdaD2 = atan2(RDP2(2), RDP2(1));

    % LOS angle time derivative (d/dt lambda), rad/s
    lambda_dot1 = (RTP1(1)*VTP1(2) - RTP1(2)*VTP1(1)) / R1^2;
    lambda_dot2 = (RTP2(1)*VTP2(2) - RTP2(2)*VTP2(1)) / R2^2;
    lambdaD_dot1 = (RDP1(1)*VDP1(2) - RDP1(2)*VDP1(1)) / RD1^2;
    lambdaD_dot2 = (RDP2(1)*VDP2(2) - RDP2(2)*VDP2(1)) / RD2^2;
    % Target heading, rad
    % beta = atan2(target.Ev, target.Nv);
    
    % Lead angle, rad
    % L = asin(Vt * sin(beta + lambda) / Vp);
    
    % True Proportional Navigation, rad/s2
    % nc = N * Vc * lambda_dot;
    
    %-------------------------------------
    % Pure Proportional Navigation
    ap1 = N * Vp1 * (lambda_dot1-w*lambdaD_dot1);
    ap2 = N * Vp2 * (lambda_dot2-w*lambdaD_dot2);
    ad  = N * Vd  * (lambdaD_dot1);
    %-------------------------------------
    
    % Terminate sim at intercept
    if abs(R1) <= 0.25 || abs(R2) <= 0.25 || abs(R12) <= 0.25 || abs(RD1) <= 0.25
        if abs(RD1) <= 0.25
            disp("Law Failure")
        elseif abs(R1) <= 0.25
            disp("success")
        
        end
        break;
    elseif k >50000/0.4-1
            disp("no outcome")
    end
    
    % Update pursuer pos for time-step and apply current slip
    pursuer1.N = pursuer1.N + pursuer1.Nv*dt + current.Nv*dt;
    pursuer1.E = pursuer1.E + pursuer1.Ev*dt + current.Ev*dt;
    pursuer2.N = pursuer2.N + pursuer2.Nv*dt + current.Nv*dt;
    pursuer2.E = pursuer2.E + pursuer2.Ev*dt + current.Ev*dt;
    % Update defender pos for time-step and apply current slip
    defender.N = defender.N + defender.Nv*dt + current.Nv*dt;
    defender.E = defender.E + defender.Ev*dt + current.Ev*dt;
    % Compute the N/E acceleration commands
    % In pure pro-nav accel commands are applied normal
    % to pursuer's velocity vector
    pNa1 = -ap1 * sin(pursuer1.psi);
    pEa1 =  ap1 * cos(pursuer1.psi);
    pNa2 = -ap2 * sin(pursuer2.psi);
    pEa2 =  ap2 * cos(pursuer2.psi);
    dNa  = -ad  * sin(defender.psi);
    dEa  =  ad  * cos(defender.psi);
    % Update pursuer N/E velocities
    pursuer1.Nv = pursuer1.Nv + pNa1*dt;
    pursuer1.Ev = pursuer1.Ev + pEa1*dt;
    pursuer2.Nv = pursuer2.Nv + pNa2*dt;
    pursuer2.Ev = pursuer2.Ev + pEa2*dt;
    defender.Nv = defender.Nv + dNa*dt;
    defender.Ev = defender.Ev + dEa*dt;
    
    % Update pursuer heading
    pursuer1.psi = atan2(pursuer1.Ev, pursuer1.Nv);
    pursuer2.psi = atan2(pursuer2.Ev, pursuer2.Nv);
    defender.psi = atan2(defender.Ev, defender.Nv);
    
    % Update target pos for time step
    if(~target.anchored)
        target.N = target.N + target.Nv*dt + current.Nv*dt;
        target.E = target.E + target.Ev*dt + current.Ev*dt;
    end
    RTP_last1 = RTP1;
    RTP_last2 = RTP2;
    RDP_last1 = RDP1;
    RDP_last2 = RDP2;
    RR12_last = RR12;
    %-------------------------------------
    % Log time-step data
    logger.t(k)   = k*dt;
    logger.pN1(k)  = pursuer1.N;
    logger.pE1(k)  = pursuer1.E;
    logger.pNv1(k) = pursuer1.Nv;
    logger.pEv1(k) = pursuer1.Ev;
    logger.psi1(k) = pursuer1.psi;
    logger.pN2(k)  = pursuer2.N;
    logger.pE2(k)  = pursuer2.E;
    logger.pNv2(k) = pursuer2.Nv;
    logger.pEv2(k) = pursuer2.Ev;
    logger.psi2(k) = pursuer2.psi;
    logger.dN1(k)  = defender.N;
    logger.dE1(k)  = defender.E;
    logger.dNv1(k) = defender.Nv;
    logger.dEv1(k) = defender.Ev;
    logger.tN(k)  = target.N;
    logger.tE(k)  = target.E;
    logger.tNv(k) = target.Nv;
    logger.tEv(k) = target.Ev;
    logger.R1(k)   = R1;
    logger.R2(k)   = R2;
    logger.Lambda1(k) = lambda1;
    logger.Lambda2(k) = lambda2;
    
end
%--------------------------------------------------------------------------
% Visualize results
%--------------------------------------------------------------------------
close all;
% % Impact index
% [M,I] = min(logger.R1);
% % Range
% %-------------------------------------
% figure;
% plot(logger.t,logger.R1); hold on;
% plot(logger.t,logger.R2); hold on;
% scatter(I*dt, M, 'filled')
% title(['Pure Proportional Navigation, N = ' num2str(N) ])
% legend('Range',...
%     ['Intercept: r = ' num2str(M) ' m, t = ' num2str(I*dt) ' s'],...
%     'Location','nw')
% ylabel('Range (m)')
% xlabel('Elapsed time (sec)')
% set(gca, 'YDir', 'reverse')
% % Heading
% %-------------------------------------
% rad2deg = @(x) x*180/pi;  % helper function
% figure;
% plot(logger.t, rad2deg(logger.psi1)); hold on;
% plot(logger.t, rad2deg(logger.Lambda1)); hold on;
% plot(logger.t, rad2deg(logger.psi2)); hold on;
% plot(logger.t, rad2deg(logger.Lambda2)); hold on;
% title(['Pure Proportional Navigation, N = ' num2str(N) ])
% legend('Pursuer1 heading', 'Pursuer2 heading', 'Bearing to target', 'Location', 'nw' )
% ylabel('Heading angle (deg)')
% xlabel('Elapsed time (sec)')
% Position
%-------------------------------------
figure;
scatter(logger.pE1, logger.pN1, 'filled'); hold on;
scatter(logger.pE2, logger.pN2, 'filled'); hold on;
scatter(logger.dE1, logger.dN1, 'filled'); hold on;
scatter(logger.tE, logger.tN, 'filled');
set(gca, 'DataAspectRatio',[10 10 10]);
title(['Pure Proportional Navigation, N = ' num2str(N) ])
legend('Pursuer1', 'Pursuer2', 'Defender', 'Target', 'Location', 'southoutside',...
    'Orientation','horizontal')
xlabel('+E (m)')
ylabel('+N (m)')