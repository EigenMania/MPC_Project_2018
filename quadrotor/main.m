clear all
close all
clc

% adding the subfolders to the path
addpath(genpath('functions'))
addpath(genpath('data'))

% loads:
%    hovering equilibrium (xs,us)
%    continuous time matrices Ac,Bc of the linearization
%    matrices sys.A, sys.B of the inner-loop discretized with sampling period sys.Ts
%    outerController optimizer instance for the outer controller
load('quadData.mat')
outerController = getOuterController(Ac);
disp('Data successfully loaded')

%%%%%%%%%%%%%%%% ADD YOUR CODE BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Part 1
%%%%%%%%%%%%%%%%%%%%%    First MPC controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART I - First MPC controller...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 5; % horizon length
z_d_max = 1;
angle_max = 10*pi/180;
angle_rate_max = 15*pi/180;
yaw_rate_max = 60*pi/180;

x1 = sdpvar(7, N);
constraints = [];
for i = 2:N
    constraints = [constraints, (-angle_max <= x1(2:3,i) <= angle_max)];
    constraints = [constraints, (-angle_rate_max <= x1(5:6,i) <= angle_max)];
    constraints = [constraints, (-z_d_max <= x1(1,i) <= z_d_max)];
    constraints = [constraints, (-yaw_rate_max <= x1(7,i) <= yaw_rate_max)];
end

u1 = sdpvar(4, N);
u_ss = (quad.mass*9.81)/(4*quad.kf);
u_min = -u_ss;
u_max = 1-u_ss;
for i = 2:N
    constraints = [constraints, (u_min <= u1(:,i) <= u_max)];
end

Q = eye(7);
R = eye(4);
P = eye(7);
objective = 0;
objective = u1(:,1)'*R*u1(:,1);
for i = 2:(N-1)
    objective = objective + x1(:,i)'*Q*x1(:,i) + u1(:,i)'*R*u1(:,i);
end
objective = objective + x1(:,N)'*P*x1(:,N);

x0 = [-1 0.1745 -0.1745 0.8727 0 0 0]';

options = sdpsettings;
innerController = optimizer(constraints, objective, options, x1(:,1), u1(:,1));
simQuad(sys, innerController, 0, x0, 5);

return
%% Part 2
%%%%%%%%%%%%%%%%%%%%%  Reference Tracking %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART II - Reference tracking...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%  First simulation of the nonlinear model %%%%%%%%%%%%%%%%%
fprintf('PART III - First simulation of the nonlinear model...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%  Offset free MPC  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART IV - Offset free MPC...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%  Simulation of the nonlinear model %%%%%%%%%%%%%%%%%%%%
fprintf('PART V - simulation of the nonlinear model...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%  Slew Rate Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART VI - Slew Rate Constraints...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%  Soft Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART VII - Soft Constraints...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  FORCES Pro %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART VIII - FORCES Pro...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
