% clean up the matlab environment
clear; clc; close all;

% run initialization of some paths and variables
init_setup;

H = 2000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% we can start working now %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Find the trim controls:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q1a: Compute the control trims values, i.e., the values of the controls
%% for which the helicopter maintains zero velocity (assuming no
%% external perturbations).
%% Note: the orientation of the helicopter will not be perfectly level: the
%% helicopter will be rolled to compensate for the tail rotor thrust.
%% Hand in trim values.

aileron_trim = -(model.params.Tx(1)) / model.params.Tx(3);
elevator_trim = 
rudder_trim =  

roll_angle_trim = -asin(model.params.Fy(1) / (model.params.m * g) );
collective_trim = 

control_trims = [ aileron_trim ; elevator_trim ; rudder_trim ; collective_trim];
quaternion_trim = [ sin(roll_angle_trim/2) ; 0 ; 0 ; cos(roll_angle_trim/2)];

target_hover_state = [ control_trims; zeros(4,1); zeros(3,1); zeros(3,1); zeros(3,1); quaternion_trim;];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulate trim controls in hover:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q1b: We now run the helicopter in open-loop, starting from the trim state.
%% We also apply the trim controls.  Up to numerical accuracy the helicopter should stay in place, as there are no perturbations.
%% Hand in plot of ned over time for the open-loop experiment below.

x(:,1) = target_hover_state;
for t=1:H
	% control law:
	delta_u = zeros(4,1); % for now just simply keep controls constant
	% simulate:
	x(:,t+1) = f_heli(x(:,t), delta_u, dt, model, idx);
end

figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('open loop');
figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('open loop');
figure;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Include state perturbation noise into the simulation:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q1c: We now include noise (external perturbations) into the simulation.
%% Turn in the ned plot and on the plot give one sentence description of what happens.

x(:,1) = target_hover_state;
for t=1:H
	% control law:
	delta_u = zeros(4,1); % for now just simply keep controls constant
	% simulate:
	noise_F_T = randn(6,1)*.1;
	x(:,t+1) = f_heli(x(:,t), delta_u, dt, model, idx, noise_F_T);
end

figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('open loop with noise');
figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('open loop with noise');
figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('open loop with noise');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Design a steady state LQR controller
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q1d: As the helicopter is open-loop unstable around hover, we design a
%% feedback controller to keep the helicopter in place.  We use LQR as
%% follows: we linearize the helicopter dynamics around the trim-state, and
%% then design the optimal feedback controller for the linearized system.
%% Then we test how well it works on the true (non-linear) system.
%% Hand in: (i) Plot of entries of K (showing convergence) 
%%          (ii) ned plot without noise
%%          (iii) ned plot with noise
%% --- note: the helicopter should be stabilized by the controller you
%% designed.

% nom_traj: we linearize around this traj
% target_traj: this is our target
% simulate_f: function that we can call as follows: next_state =
%            simulate_f(current_state, inputs, simtime, params, model_bias)
% model: parameters and features of the dynamics model (simulate_f)
% idx: how we index into features and state using named indexes

u_prev_mult = 0; u_delta_prev_mult = 1000;
ned_dot_mult = 1; ned_mult = 1;
pqr_mult = 1; q_mult = 1;
always1state_mult = 0;

reward.state_multipliers = [ u_prev_mult * ones(1,4)   u_delta_prev_mult * ones(1,4)  ned_dot_mult * ones(1,3)  ned_mult*ones(1,3)  ...
	pqr_mult * ones(1,3)  q_mult * ones(1,3)  always1state_mult * ones(1,1)]';
reward.input_multipliers = ones(4,1)*0;


Ps = zeros(length(reward.state_multipliers));

nominal_state = target_hover_state; nominal_inputs = zeros(4,1);
target_state_time_t = target_hover_state; target_state_time_tplus1 = target_hover_state;
magic_factor = 0;
model_bias = zeros(6,1);
simulate_f = @f_heli;

[A, B] = linearized_dynamics(nominal_state, nominal_inputs, ...
	target_state_time_t, target_state_time_tplus1, ...
	simulate_f, dt, model, idx, model_bias, magic_factor, target_state_time_tplus1);

Q = diag(reward.state_multipliers) * dt;
R = diag(reward.input_multipliers) * dt;

num_steps = % make sure it is sufficient for K to have converged
for i=1:num_steps
	K{i} = 

	Ps = 
end

figure; hold on;
for k=1:size(K{1},1)
	for l=1:size(K{1},2)
		for i=1:num_steps
			K_entry_to_plot(i) = K{i}(k,l);
		end
		plot(K_entry_to_plot);
	end
end

K_ss = K{num_steps}; % the steady state (infinite horizon) feedback matrix



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% sanity check in simulation without noise:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x(:,1) = target_hover_state;
for t=1:H
	% control law:
	dx = compute_dx(target_hover_state, x(:,t));
	delta_u = K_ss*dx;
	% simulate:
	x(:,t+1) = f_heli(x(:,t), delta_u, dt, model, idx);
end
figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('K_{ss} hover');
figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('K_{ss} hover');
figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('K_{ss} hover');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% simulate with noise:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x(:,1) = target_hover_state;
for t=1:H
	% control law:
	dx = compute_dx(target_hover_state, x(:,t));
	% state observation noise:
	v = randn(size(dx,1)-1,1)*.1;
	dx(1:end-1) = dx(1:end-1) + v;
	delta_u = K_ss* dx;
	% simulate:
	noise_F_T = randn(6,1)*1;
	x(:,t+1) = f_heli(x(:,t), delta_u, dt, model, idx, noise_F_T);
end
figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('K_{ss} hover with noise');
figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('K_{ss} hover with noise');
figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('K_{ss} hover with noise');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller was designed by linearization around hover trim condition,
%% let's see what happens when we get further away from the linearization:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q1e: Now let's see how far away from the linearization point we can
%% start, and still have a good controller.
%% Hand in: (i) the smallest (positive) value for north, east and down that
%% makes the controller perform poorly [granularity of 10m]
%%          (ii) the smallest rotation around x, y, and z that makes the
%%          controller perform poorly

start_ned = [0; 0; 0];
rotation_axis = [0; 0; 0];
rotation_angle = 0; % radians
start_q = [sin(rotation_angle/2)*rotation_axis; cos(rotation_angle/2)];

start_state = target_hover_state;
start_state(idx.ned) = start_ned;
start_state(idx.q) = start_q;


x(:,1) = start_state;
for t=1:H
	% control law:
	dx = compute_dx(target_hover_state, x(:,t));
	delta_u = K_ss*dx;
	% simulate:
	x(:,t+1) = f_heli(x(:,t), delta_u, dt, model, idx);
end
figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('K_{ss} hover with perturbed initial state');
figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('K_{ss} hover with perturbed initial state');
figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('K_{ss} hover with perturbed initial state');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% clip errors to deal with large errors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q1f: we start at north = 10000, for which the linear controller ends up
%% performing quite poorly.  Pick a clipping distance that makes the
%% controller perform well---the clipping distance maxes out errors at that
%% value.  Hand in: plot of ned + the clipping distance you picked.  

start_ned = [10000; 0; 0];
rotation_axis = [0; 0; 1];
rotation_angle = 0; % radians
start_q = [sin(rotation_angle/2)*rotation_axis; cos(rotation_angle/2)];

start_state = target_hover_state;
start_state(idx.ned) = start_ned;
start_state(idx.q) = start_q;


clipping_distance =  %% your pick
x(:,1) = start_state;
for t=1:H
	% control law:
	dx = compute_dx(target_hover_state, x(:,t));
	dx(idx.ned) = max(min(dx(idx.ned), clipping_distance),-clipping_distance);
	delta_u = K_ss*dx;
	% simulate:
	noise_F_T = randn(6,1)*1;
	x(:,t+1) = f_heli(x(:,t), delta_u, dt, model, idx, noise_F_T);
end
figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('K_{ss} hover with perturbed initial state and clipping');
figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('K_{ss} hover with perturbed initial state and clipping');
figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('K_{ss} hover with perturbed initial state and clipping');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% simulate with latency:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q1f: latency tends to be a killer for many controllers
%% find out the smallest (discrete) latency that makes the controller
%% perform poorly; hand in plot of q(uaternion) and u_prev + the latency value

latency = %% your pick
for i=1:latency+1
	x(:,i) = target_hover_state;
end

for t=latency+1:H
	% control law:
	dx = compute_dx(target_hover_state, x(:,t-latency));
	delta_u = K_ss* dx;
	% simulate:
	noise_F_T = randn(6,1)*1;
	x(:,t+1) = f_heli(x(:,t), delta_u, dt, model, idx, noise_F_T);
end
figure; plot(x(idx.ned,:)'); legend('north', 'east', 'down'); title('K_{ss} hover with latency');
figure; plot(x(idx.q,:)'); legend('qx', 'qy', 'qz', 'qw'); title('K_{ss} hover with latency');
figure; plot(x(idx.u_prev,:)'); legend('aileron','elevator','rudder','collective'); title('K_{ss} hover with latency');

%% Q1g: Build a controller about the hover 
%% configuration by performing search 
%% over a space of policies. Use the 'real' non-linear model, not linearized dynamics.  
%% We suggest starting with a simple linear controller class, but you might
%% consider somewhat more sophisticated ones then that.

%% Q1h: Build the same controller as above but now build it using the latency described above. Can you build a linear
%% controller more robust to various latencies then the LQR one? Plot the range of latencies for which the LQR and
%% policy search controller can stabilize the helicopter.





end
