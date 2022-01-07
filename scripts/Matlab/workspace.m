close all; clear; clc; 
% Robot links
L1 = Link([pi 0.09766 0.266 -pi/2],'standard');
L2 = Link([-pi/2 0 0.53925 0], 'standard');
L3 = Link([pi/2 0 0.45135 0 ], 'standard');
L4 = Link([pi/2 -0.00098 0 -pi/2 ],'standard');
L1.qlim =[0,2*pi];
L2.qlim =[(-0.1745-pi/2),(1.9198-pi/2)];
L3.qlim =[pi/2-0.1745,pi/2+1.3962];
L4.qlim =[-1.0472,pi/2];
base = [1 0 0 0; 0 1 0 0; 0 0 1 0.099665;0 0 0 1];
tool = [1 0 0 0.1462+0.13705; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% Create robot
Robot=SerialLink([L1 L2 L3 L4], 'name', 'Zeus Arm','base',base, 'tool',tool);
Robot.teach([0 -pi/2 pi/2 0]);
hold on;
% Define robot limits 
N = 30000;
limitmax_1 = 2*pi;
limitmin_1 = 0.0;
limitmax_2 = (1.9198-pi/2);
limitmin_2 = (-0.1745-pi/2);
limitmax_3 = pi/2+1.3962;
limitmin_3 = pi/2-0.1745;
limitmax_4 = pi/2;
limitmin_4 = -1.0472;
% Random points in robot range
theta1=(limitmin_1+(limitmax_1-limitmin_1)*rand(N,1)); 
theta2=(limitmin_2+(limitmax_2-limitmin_2)*rand(N,1)); 
theta3=(limitmin_3+(limitmax_3-limitmin_3)*rand(N,1)); 
theta4=(limitmin_4+(limitmax_4-limitmin_4)*rand(N,1)); 
% Forward kinematics
qq = [theta1,theta2,theta3,theta4];
T = double(Robot.fkine(qq));
x=reshape(T(1,4,:),N,1);
y=reshape(T(2,4,:),N,1);
z=reshape(T(3,4,:),N,1);
% Plot robot workspace
plot3(x,y,z,'b.','MarkerSize',0.5);
title('Robot workspace');
hold on;



% for n=1:1:N
%     qq = [theta1(n),theta2(n),theta3(n),theta4(n)];
%     Robot.plot(qq);
%     T = double(Robot.fkine(qq));
%     plot3(T(1,4,:), T(2,4,:), T(3,4,:), 'b.', 'MarkerSize', 0.5);
%     hold on;
% end

