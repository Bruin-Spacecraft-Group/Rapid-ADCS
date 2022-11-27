%% Orientation Test Sim
clc; clear variables; close all;

%% Initialize satellite parameters
I_w = 0.5 * 0.05*0.05^2;            % [kg m^2]      Individual reaction wheel inertia about rot. axis
Ix = 0.03; Iy = 0.04; Iz = 0.07;    % [kg m^2]      Satellite total principal inertias
I_s = [Ix 0 0; 0 Iy 0; 0 0 Iz];     % [kg m^2]      Satellite total inertia matrix
inv_I_s = inv(I_s);                 % [kg^-1 m^-2]  Inverse total inertia matrix

K = 1;         % [] arbitrary positive control parameter
lambda = 1;    % [] arbitrary positive control parameter

%% Initialize simulation parameters
t0 = 0; tf = 60;            % [s] Simulation initial, final time 
dt = 0.01;                  % [s] Simulation timestep
N = (tf-t0)/dt + 1;         % [ ] Simulation step count
t = linspace(t0, tf, N);    % [s] Time vector

%% Initial conditions
% Initial state
psi_0 = deg2rad(0);                     % [rad]   Initial heading
theta_0 = deg2rad(0);                   % [rad]   Initial pitch
phi_0 = deg2rad(0);                     % [rad]   Initial roll
qb0 = TB2quat(psi_0, theta_0, phi_0);   % [ ]     Initial orientation quaternion
wb0 = [0; 0; 0];                        % [rad/s] Initial angular velocity
ww0 = [0; 0; 0];                        % [rad/s] Initial wheel angular velocity

% Angle command
psi_c = deg2rad(180);                   % [rad]   Command heading
theta_c = deg2rad(0);                   % [rad]   Command pitch
phi_c = deg2rad(0);                     % [rad]   Command roll
qc = TB2quat(psi_c, theta_c, phi_c);    % [ ]     Command quaternion

% Simulation variables
qb = zeros(4,N);                        % [ ]     Body orientation quaternion
qe = zeros(4,N);                        % [ ]     Error quaternion
qb_dot = zeros(4,N);                    % [s^-1]  Body orientation quaternion derivative
qe_dot = zeros(4,N);                    % [s^-1]  Error quaternion derivative

wb = zeros(3,N);                        % [s^-1]  Body rate
ww = zeros(3,N);                        % [s^-1]  Reaction wheel speeds
wb_dot = zeros(3,N);                    % [s^-2]  Body ang. acceleration
ww_dot = zeros(3,N);                    % [s^-2]  Reaction wheel ang. accelerations

% Initialize simulation variables???
ww(:,1) = ww0;
wb(:,1) = wb0;
qb(:,1) = qb0;

%% Simulate dynamics
for i = 1:N-1
    
    % Control: Calculate reaction wheel torque
    qe(:,i) = quatmult(inv_q(qc), qb(:,i));

    wr = [0;0;0];
    we = wb(:,i) - wr; % angular velocity error vector

    qe_dot(:,i) = 0.5*quatmult(qe(:,i), quat(we));
    wr_dot = [0;0;0];

    H_s = I_s*wb(:,i);
    H_w = I_w*ww(:,i); % assume aligned orthogonal wheels, each wheel rot. symmetric

    s = we + lambda*sgn(qe(1,i))*qe(2:4,i); % sliding variable

    tau_w = K*I_s*s - cross(wb(:,i),H_s+H_w) - I_s*wr_dot + I_s*lambda*sgn(qe(1,i))*qe_dot(2:4,i);
    ww_dot(:,i) = tau_w./I_w;

    % Dynamic equations
    wb_dot(:,i) = inv_I_s*(-tau_w - cross(wb(:,i), H_s+H_w));
    qb_dot(:,i) = 0.5*quatmult(qb(:,i), quat(wb(:,i)));

    % Euler method
    ww(:,i+1) = ww(:,i) + ww_dot(:,i)*dt;
    wb(:,i+1) = wb(:,i) + wb_dot(:,i)*dt;
    qb(:,i+1) = qb(:,i) + qb_dot(:,i)*dt;
    qb(:,i+1) = qb(:,i+1)/norm(qb(:,i+1)); % Re-normalize quaternion
end

%% Recover quantities of interest
% Euler angles
psi = zeros(1,N);
theta = zeros(1,N);
phi = zeros(1,N);
psi_last = psi_0;
for i = 1:N
    qw = qb(1,i);
    qx = qb(2,i);
    qy = qb(3,i);
    qz = qb(4,i);

    theta(i) = -asin(2*(qx*qz - qw*qy));
    if theta(i) == pi/2
        psi(i) = psi_last;
        phi(i) = atan2(2*(qx*qy-qw*qz), 1-2*(qx^2 + qz^2)) + psi(i);
    elseif theta(i) == -pi/2
        psi(i) = psi_last;
        phi(i) = atan2(2*(qw*qz-qx*qy), 1-2*(qx^2 + qz^2)) - psi(i);
    else
        psi(i) = atan2(2*(qx*qy+qw*qz), 1 - 2*(qy^2 + qz^2));
        phi(i) = atan2(2*(qw*qx+qy*qz), 1 - 2*(qx^2 + qy^2));
    end
    psi_last = psi(i);
end
% Convert to degrees
psi = rad2deg(psi);
theta = rad2deg(theta);
phi = rad2deg(phi);

% Plot
figure(1)
plot(t,psi,t,theta,t,phi)
legend('\Psi','\Theta','\Phi')

figure(2)
plot(t,ww)
legend('\Omega_1','\Omega_2','\Omega_3')

%% Utility functions
function q = TB2quat(psi, theta, phi)
% converts Tait-Bryan angles to quaternion
    cy = cos(psi/2);    sy = sin(psi/2);    % Trig yaw terms
    cp = cos(theta/2);  sp = sin(theta/2);  % Trig pitch terms
    cr = cos(phi/2);    sr = sin(phi/2);    % Trig roll terms

    q = [cr*cp*cy + sr*sp*sy;
         sr*cp*cy - cr*sp*sy;
         cr*sp*cy + sr*cp*sy;
         cr*cp*sy - sr*sp*cy];
end

function q_inv = inv_q(q)
    q_inv = [-q(1);
             q(2:4)];
end

function quat_out = quat(vec_in)
    quat_out = [0; vec_in];
end

function out = sgn(in)
    out = sign(in);
    if in == 0
        out = 1;
    end
end