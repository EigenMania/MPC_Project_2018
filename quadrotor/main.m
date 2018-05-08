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
N = 30; % horizon length
z_d_max = 1;
angle_max = 10*pi/180;
angle_rate_max = 15*pi/180;
yaw_rate_max = 60*pi/180;

n_states = 7;
n_inputs = 4;

x1 = sdpvar(n_states, N);
u1 = sdpvar(n_inputs, N);

constraints = [];
for i = 2:N
    % dynamics constraints
    constraints = [constraints, x1(:,i) == sys.A*x1(:,i-1) + sys.B*u1(:,i-1)];
    
    % state constraints
    constraints = [constraints, (-angle_max <= x1(2:3,i) <= angle_max)];
    constraints = [constraints, (-angle_rate_max <= x1(5:6,i) <= angle_max)];
    constraints = [constraints, (-z_d_max <= x1(1,i) <= z_d_max)];
    constraints = [constraints, (-yaw_rate_max <= x1(7,i) <= yaw_rate_max)];
end

u_ss = (quad.mass*9.81)/(4*quad.kf);
u_min = -u_ss;
u_max = 1-u_ss;
for i = 1:(N-1)
    % input constraints
    constraints = [constraints, (u_min <= u1(:,i) <= u_max)];
end

% NOTE: It seems that when the horizon length is 5 and the cost for the
% angles is increased past 1, the problem becomes unfeasible. Why on earth
% does this happen?!?!?!?!?!?
Q = diag([1 150 150 1 1 1 1]);
R = eye(4);
objective = 0;
for i = 1:(N-1)
    % objective function
    objective = objective +  u1(:,i)'*R*u1(:,i);
end

for i = 2:(N-1)
    % objective function
    objective = objective +  x1(:,i)'*Q*x1(:,i);
end

x0 = [-1 0.1745 -0.1745 0.8727 0 0 0]';

model = LTISystem('A', sys.A, 'B', sys.B, 'Ts', sys.Ts);
model.x.min = [-z_d_max -angle_max -angle_max -Inf -angle_rate_max -angle_rate_max -yaw_rate_max]';
model.x.max = [z_d_max   angle_max  angle_max  Inf  angle_rate_max  angle_rate_max  yaw_rate_max]';
model.x.penalty = QuadFunction(Q);

model.u.min = u_min*ones(4,1);
model.u.max = u_max*ones(4,1);
model.u.penalty = QuadFunction(R);

Tset = model.LQRSet;
PN = model.LQRPenalty;

constraints = [constraints (Tset.A*x1(:,N) <= Tset.b)];
objective = objective + x1(:,N)'*PN.weight*x1(:,N);

options = sdpsettings;
innerController = optimizer(constraints, objective, options, x1(:,1), u1(:,1));
simQuad(sys, innerController, 0, x0, 5);

return
%% Part 2
%%%%%%%%%%%%%%%%%%%%%  Reference Tracking %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART II - Reference tracking...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = sdpvar(4,1); % reference designed as dummy variable
C = [eye(4,4) zeros(4,3)];
M = [ eye(size(sys.A))-sys.A -sys.B ; C zeros(size(C,1), size(sys.B,2))];

ss = M\[zeros(n_states, 1) ; r]; % NOTE: state makes sense, all ss inputs zero because of linear model
x_r = ss(1:n_states);
u_r = ss(n_states+1:end);

% Initial hard code...
%x_r = [r ; zeros(3,1)];
%u_r = zeros(4,1);

constraints = [];
for i = 2:N
    % dynamics constraints
    constraints = [constraints, x1(:,i) == sys.A*x1(:,i-1) + sys.B*u1(:,i-1)];
    
    % state constraints
    constraints = [constraints, (-angle_max <= x1(2:3,i) <= angle_max)];
    constraints = [constraints, (-angle_rate_max <= x1(5:6,i) <= angle_max)];
    constraints = [constraints, (-z_d_max <= x1(1,i) <= z_d_max)];
    constraints = [constraints, (-yaw_rate_max <= x1(7,i) <= yaw_rate_max)];
end

for i = 1:(N-1)
    % input constraints
    constraints = [constraints, (u_min <= u1(:,i) <= u_max)];
end

objective = 0;
for i = 1:(N-1)
    % objective function
    objective = objective +  (u1(:,i)-u_r)'*R*(u1(:,i)-u_r);
end

for i = 2:(N-1)
    % objective function
    objective = objective +  (x1(:,i)-x_r)'*Q*(x1(:,i)-x_r);
end
%objective = objective + x1(:,N)'*PN.weight*x1(:,N); % NOTE: This makes things worse...

x0 = [0 0 0 0 0 0 0]';
r1 = [1.0 ; 0.1745 ; -0.1745 ; 1.7453];

T = 10;
options = sdpsettings;
innerController = optimizer(constraints, objective, options, [x1(:,1);r], u1(:,1));
simQuad(sys, innerController, 0, x0, T, r1);

%% Slowly varying reference signal
t_vec = 0:sys.Ts:T;
r_k = [ones(1,length(t_vec)) ; 0.1745*sin(t_vec) ; -0.1745*sin(t_vec) ; ones(1,length(t_vec))*pi/2];

x0 = [0 0 0 0 0 0 0]';
T = 10;
options = sdpsettings;
innerController = optimizer(constraints, objective, options, [x1(:,1) ; r], u1(:,1));
simQuad(sys, innerController, 0, x0, T, r_k);


return
%% Part 3
%%%%%%%%%%%%%%%  First simulation of the nonlinear model %%%%%%%%%%%%%%%%%
fprintf('PART III - First simulation of the nonlinear model...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sim('simulation1.mdl');

return
%% Part 4
%%%%%%%%%%%%%%%%%%%%%%%  Offset free MPC  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('PART IV - Offset free MPC...\n')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d = sdpvar(7,1); % reference designed as dummy variable

L = zeros(14,7);
C = eye(7); % observe entire state
Aaug = [sys.A eye(7) ; zeros(7,7) eye(7)];
Baug = [sys.B ; zeros(7,4)];
Caug = [eye(7) zeros(7,7)];
P = [0.85*ones(7,1) ; 0.9*ones(7,1)];

L = place(Aaug', -Caug', P)';

Af = [Aaug + L*Caug];
Bf = [Baug -L];

filter.Af = Af;
filter.Bf = Bf;

C = [eye(4,4) zeros(4,3)];
M = [eye(size(sys.A))-sys.A -sys.B ; C zeros(size(C,1), size(sys.B,2))];

ss = M\[d ; r]; % NOTE: state makes sense, all ss inputs zero because of linear model
x_r = ss(1:n_states);
u_r = ss(n_states+1:end);

constraints = [];
for i = 2:N
    % dynamics constraints
    constraints = [constraints, x1(:,i) == sys.A*x1(:,i-1) + sys.B*u1(:,i-1) + d];
    
    % state constraints
    constraints = [constraints, (-angle_max <= x1(2:3,i) <= angle_max)];
    constraints = [constraints, (-angle_rate_max <= x1(5:6,i) <= angle_max)];
    constraints = [constraints, (-z_d_max <= x1(1,i) <= z_d_max)];
    constraints = [constraints, (-yaw_rate_max <= x1(7,i) <= yaw_rate_max)];
end

for i = 1:(N-1)
    % input constraints
    constraints = [constraints, (u_min <= u1(:,i) <= u_max)];
end

objective = 0;
for i = 1:(N-1)
    % objective function
    objective = objective +  (u1(:,i)-u_r)'*R*(u1(:,i)-u_r);
end

for i = 2:(N-1)
    % objective function
    objective = objective +  (x1(:,i)-x_r)'*Q*(x1(:,i)-x_r);
end

x0 = [0 0 0 0 0 0 0]';
r1 = [0.8 ; 0.12 ; -0.12 ; pi/2];

T = 10;
options = sdpsettings;
innerController = optimizer(constraints, objective, options, [x1(:,1);r;d], u1(:,1));
simQuad(sys, innerController, 0, x0, T, r1, filter);

return
%% Slowly varying reference signal
t_vec = 0:sys.Ts:T;
r_k = [0.8*ones(1,length(t_vec)) ; 0.12*sin(t_vec) ; -0.12*sin(t_vec) ; ones(1,length(t_vec))*pi/2];

x0 = [0 0 0 0 0 0 0]';
T = 10;
options = sdpsettings;
innerController = optimizer(constraints, objective, options, [x1(:,1) ; r ; d], u1(:,1));
simQuad(sys, innerController, 0, x0, T, r_k, filter);

return


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
