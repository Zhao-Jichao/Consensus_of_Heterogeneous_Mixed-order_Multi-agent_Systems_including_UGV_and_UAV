% UGV/UAV 异构控制一致性控制
% Author: Zhao-Jichao
% Date: 2021-05-14
clear
clc


%% Laplacian Matrix
L = [2 -1 -1  0  0
    -1  3 -1 -1  0
    -1 -1  3  0 -1
     0 -1  0  2 -1
     0  0 -1 -1  2];


%% Initial States
% PositionX, PositionY, VeloctiyX, VelocityY (4)
% UGV1 = [90, 50,  10, 10]';
% UGV2 = [65, 10,  10, 10]';
% UGV3 = [55, 20,  10, 10]';
% PositionX, PositionY, PositionZ, VeloctiyX, VelocityY, VelocityZ,
% Pitch, Phi, Psi, dPitch, dPhi, dPsi (12)
% UAV4 = [25, 10, 10,  10, 10, 00,  10, 10, 00,  10, 10, 00]';
% UAV5 = [10, 30, 10,  10, 10, 00,  10, 10, 00,  10, 10, 00]';

% X0 = [UGV1; UGV2; UGV3; UAV4; UAV5];

P1X(1,1) = 90; P1Y(1,1) = 50; V1X(1,1) = 0; V1Y(1,1) = 15;
P2X(1,1) = 65; P2Y(1,1) = 10; V2X(1,1) = 10; V2Y(1,1) = 0;
P3X(1,1) = 55; P3Y(1,1) = 20; V3X(1,1) = 20; V3Y(1,1) = 15;

P4X(1,1) = 30;       P4Y(1,1) = 50;     P4Z(1,1) = 50;
V4X(1,1) = 00;       V4Y(1,1) = 10;     V4Z(1,1) = 0;
gtheta4(1,1) = 10;   gphi4(1,1) = 30;   gpsi4(1,1) = 0;
gdtheta4(1,1) = 00;  gdphi4(1,1) = 00;  gdpsi4(1,1) = 0;

P5X(1,1) = 90;       P5Y(1,1) = 30;     P5Z(1,1) = 50;
V5X(1,1) = 00;       V5Y(1,1) = 10;     V5Z(1,1) = 0;
gtheta5(1,1) = 20;   gphi5(1,1) = 15;   gpsi5(1,1) = 00;
gdtheta5(1,1) = 00;  gdphi5(1,1) = 00;  gdpsi5(1,1) = 00;

% Time parameters
tBegin = 0;
tFinal = 50;
dT = 0.01;
times = (tFinal-tBegin)/dT;
time(1,1) = 0;


%% Calculate
r1 = 0.1;
r2 = 0.8;
r3 = 4.0;
r4 = 1.5;
global protocol
protocol = 1;   % (1:Dynamic/2:Static)

for i=1:times
    time(i+1,1) = time(i,1) + dT;
    if protocol == 1
    u1x = r1*( (P2X(i,:)-P1X(i,:))+(P3X(i,:)-P1X(i,:)) ) + r2*( (V2X(i,:)-V1X(i,:))+(V3X(i,:)-V1X(i,:)) );
    u2x = r1*( (P1X(i,:)-P2X(i,:))+(P3X(i,:)-P2X(i,:)+(P4X(i,:)-P2X(i,:))) ) + r2*( (V1X(i,:)-V2X(i,:))+(V3X(i,:)-V2X(i,:))+(V4X(i,:)-V2X(i,:)) );
    u3x = r1*( (P1X(i,:)-P3X(i,:))+(P2X(i,:)-P3X(i,:)+(P5X(i,:)-P3X(i,:))) ) + r2*( (V1X(i,:)-V3X(i,:))+(V2X(i,:)-V3X(i,:))+(V5X(i,:)-V3X(i,:)) );
    u4x = r1*( (P2X(i,:)-P4X(i,:))+(P5X(i,:)-P4X(i,:)) ) + r2*( (V2X(i,:)-V4X(i,:))+(V5X(i,:)-V4X(i,:)) ) + r3*( -gtheta4(i,:)) + r4*( -gdtheta4(i,:));
    u5x = r1*( (P3X(i,:)-P5X(i,:))+(P4X(i,:)-P5X(i,:)) ) + r2*( (V3X(i,:)-V5X(i,:))+(V4X(i,:)-V5X(i,:)) ) + r3*( -gtheta5(i,:)) + r4*( -gdtheta5(i,:));
    u1y = r1*( (P2Y(i,:)-P1Y(i,:))+(P3Y(i,:)-P1Y(i,:)) ) + r2*( (V2Y(i,:)-V1Y(i,:))+(V3Y(i,:)-V1Y(i,:)) );
    u2y = r1*( (P1Y(i,:)-P2Y(i,:))+(P3Y(i,:)-P2Y(i,:)+(P4Y(i,:)-P2Y(i,:))) ) + r2*( (V1Y(i,:)-V2Y(i,:))+(V3Y(i,:)-V2Y(i,:))+(V4Y(i,:)-V2Y(i,:)) );
    u3y = r1*( (P1Y(i,:)-P3Y(i,:))+(P2Y(i,:)-P3Y(i,:)+(P5Y(i,:)-P3Y(i,:))) ) + r2*( (V1Y(i,:)-V3Y(i,:))+(V2Y(i,:)-V3Y(i,:))+(V5Y(i,:)-V3Y(i,:)) );
    u4y = r1*( (P2Y(i,:)-P4Y(i,:))+(P5Y(i,:)-P4Y(i,:)) ) + r2*( (V2Y(i,:)-V4Y(i,:))+(V5Y(i,:)-V4Y(i,:)) ) + r3*( -gphi4(i,:)) + r4*( -gdphi4(i,:));
    u5y = r1*( (P3Y(i,:)-P5Y(i,:))+(P4Y(i,:)-P5Y(i,:)) ) + r2*( (V3Y(i,:)-V5Y(i,:))+(V4Y(i,:)-V5Y(i,:)) ) + r3*( -gphi5(i,:)) + r4*( -gdphi5(i,:));
    end
    if protocol == 2
    u1x = r1*( (P2X(i,:)-P1X(i,:))+(P3X(i,:)-P1X(i,:)) ) + r2*( -V1X(i,:) );
    u2x = r1*( (P1X(i,:)-P2X(i,:))+(P3X(i,:)-P2X(i,:)+(P4X(i,:)-P2X(i,:))) ) + r2*( -V2X(i,:) );
    u3x = r1*( (P1X(i,:)-P3X(i,:))+(P2X(i,:)-P3X(i,:)+(P5X(i,:)-P3X(i,:))) ) + r2*( -V3X(i,:) );
    u4x = r1*( (P2X(i,:)-P4X(i,:))+(P5X(i,:)-P4X(i,:)) ) + r2*( (V2X(i,:)-V4X(i,:))+(V5X(i,:)-V4X(i,:)) ) + r3*( -gtheta4(i,:)) + r4*( -gdtheta4(i,:));
    u5x = r1*( (P3X(i,:)-P5X(i,:))+(P4X(i,:)-P5X(i,:)) ) + r2*( (V3X(i,:)-V5X(i,:))+(V4X(i,:)-V5X(i,:)) ) + r3*( -gtheta5(i,:)) + r4*( -gdtheta5(i,:));
    u1y = r1*( (P2Y(i,:)-P1Y(i,:))+(P3Y(i,:)-P1Y(i,:)) ) + r2*( -V1Y(i,:) );
    u2y = r1*( (P1Y(i,:)-P2Y(i,:))+(P3Y(i,:)-P2Y(i,:)+(P4Y(i,:)-P2Y(i,:))) ) + r2*( -V2Y(i,:) );
    u3y = r1*( (P1Y(i,:)-P3Y(i,:))+(P2Y(i,:)-P3Y(i,:)+(P5Y(i,:)-P3Y(i,:))) ) + r2*( -V3Y(i,:) );
    u4y = r1*( (P2Y(i,:)-P4Y(i,:))+(P5Y(i,:)-P4Y(i,:)) ) + r2*( (V2Y(i,:)-V4Y(i,:))+(V5Y(i,:)-V4Y(i,:)) ) + r3*( -gphi4(i,:)) + r4*( -gdphi4(i,:));
    u5y = r1*( (P3Y(i,:)-P5Y(i,:))+(P4Y(i,:)-P5Y(i,:)) ) + r2*( (V3Y(i,:)-V5Y(i,:))+(V4Y(i,:)-V5Y(i,:)) ) + r3*( -gphi5(i,:)) + r4*( -gdphi5(i,:));
    end

    
    V1X(i+1,1) = V1X(i,1)+dT*u1x; P1X(i+1,1) = P1X(i,1)+dT*V1X(i+1,1);
    V2X(i+1,1) = V2X(i,1)+dT*u2x; P2X(i+1,1) = P2X(i,1)+dT*V2X(i+1,1);
    V3X(i+1,1) = V3X(i,1)+dT*u3x; P3X(i+1,1) = P3X(i,1)+dT*V3X(i+1,1);
    
    gdtheta4(i+1,1) = gdtheta4(i,1)+dT*u4x; 
    gtheta4(i+1,1) = gtheta4(i,1)+dT*gdtheta4(i+1,1); 
    V4X(i+1,1) = V4X(i,1)+dT*gtheta4(i+1,1); 
    P4X(i+1,1) = P4X(i,1)+dT*V4X(i+1,1);
    
    gdtheta5(i+1,1) = gdtheta5(i,1)+dT*u5x; 
    gtheta5(i+1,1) = gtheta5(i,1)+dT*gdtheta5(i+1,1); 
    V5X(i+1,1) = V5X(i,1)+dT*gtheta5(i+1,1); 
    P5X(i+1,1) = P5X(i,1)+dT*V5X(i+1,1);
    
    V1Y(i+1,1) = V1Y(i,1)+dT*u1y; P1Y(i+1,1) = P1Y(i,1)+dT*V1Y(i+1,1);
    V2Y(i+1,1) = V2Y(i,1)+dT*u2y; P2Y(i+1,1) = P2Y(i,1)+dT*V2Y(i+1,1);
    V3Y(i+1,1) = V3Y(i,1)+dT*u3y; P3Y(i+1,1) = P3Y(i,1)+dT*V3Y(i+1,1);
    
    gdphi4(i+1,1) = gdphi4(i,1)+dT*u4y; 
    gphi4(i+1,1) = gphi4(i,1)+dT*gdphi4(i+1,1); 
    V4Y(i+1,1) = V4Y(i,1)+dT*gphi4(i+1,1); 
    P4Y(i+1,1) = P4Y(i,1)+dT*V4Y(i+1,1);
    
    gdphi5(i+1,1) = gdphi5(i,1)+dT*u5y; 
    gphi5(i+1,1) = gphi5(i,1)+dT*gdphi5(i+1,1); 
    V5Y(i+1,1) = V5Y(i,1)+dT*gphi5(i+1,1); 
    P5Y(i+1,1) = P5Y(i,1)+dT*V5Y(i+1,1);
end


%% Draw graphs
subplot(4,2,1, 'Position',[0.02 0.775 0.45 0.20])
plot(time,P1X, time,P2X, time,P3X, time,P4X, time,P5X, 'linewidth',1.5);
legend('P^X_{G1}','P^X_{G2}','P^X_{G3}','P^X_{A1}','P^X_{A2}'); grid on
title("X Positions")

subplot(4,2,3, 'Position',[0.02 0.525 0.45 0.20])
plot(time,V1X, time,V2X, time,V3X, time,V4X, time,V5X, 'linewidth',1.5);
legend('V^X_{G1}','V^X_{G2}','V^X_{G3}','V^X_{A1}','V^X_{A2}'); grid on
title("X Velocities")

subplot(4,2,5, 'Position',[0.02 0.275 0.45 0.20])
plot(time,gtheta4, time,gtheta5, 'linewidth',1.5);
legend('theta_{A1}','theta_{A2}'); grid on
title("X Omega")

subplot(4,2,7, 'Position',[0.02 0.025 0.45 0.20])
plot(time,gdtheta4, time,gdtheta5, 'linewidth',1.5);
legend('dtheta_{A1}','dtheta_{A2}'); grid on
title("X dotOmega")


subplot(4,2,2, 'Position',[0.53 0.775 0.45 0.20])
plot(time,P1Y, time,P2Y, time,P3Y, time,P4Y, time,P5Y, 'linewidth',1.5);
legend('P^Y_{G1}','P^Y_{G2}','P^Y_{G3}','P^Y_{A1}','P^Y_{A2}'); grid on
title("Y Positions")

subplot(4,2,4, 'Position',[0.53 0.525 0.45 0.20])
plot(time,V1Y, time,V2Y, time,V3Y, time,V4Y, time,V5Y, 'linewidth',1.5);
legend('V^Y_{G1}','V^Y_{G2}','V^Y_{G3}','V^Y_{A1}','V^Y_{A2}'); grid on
title("Y Velocities")

subplot(4,2,6, 'Position',[0.53 0.275 0.45 0.20])
plot(time,gphi4, time,gphi5, 'linewidth',1.5);
legend('phi_{A1}','phi_{A2}'); grid on
title("Y Omega")

subplot(4,2,8, 'Position',[0.53 0.025 0.45 0.20])
plot(time,gdphi4, time,gdphi5, 'linewidth',1.5);
legend('dphi_{A1}','dphi_{A2}'); grid on
title("Y dotOmega")



