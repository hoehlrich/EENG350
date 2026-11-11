%% build_models.m
% Builds the Simulink model of the Demo 2 run, so it can be rebuilt exactly
% and the structure is readable in this file. The built .slx is committed
% too, so this only needs to run after a change to the structure.
%
%   beacon_approach.slx   The SteeringControl handout section 6 diagram
%                         ("moving towards an object") around Demo 1's two
%                         control channels:
%
%     Camera       beacon_camera.m: angle and distance from the camera to
%                  the beacon, from the robot's x, y and phi, with a field
%                  of view and a frame rate. A Delay block after it is the
%                  time a reading takes to reach the Arduino.
%     Supervisor   beacon_supervisor.m: the stage logic of beacon.ino
%                  (search, face, approach, turn). Turns the camera
%                  readings into targets and steps the reference profiles.
%     Forward,     Demo 1's channels: outer PI position loop around an
%     Rotation     inner speed loop, for rho and for phi.
%     x_dot, y_dot x_dot = rho_dot*cos(phi), y_dot = rho_dot*sin(phi),
%                  integrated for the path, which is what the camera sees.
%
% In the handout the camera angle phi_s goes straight into the heading
% controller. Here it goes through the Supervisor, which adds it to the
% odometry heading to make a target (phi_des = phi + phi_s, the same
% equation) and then feeds that target through a trapezoidal profile.
%
% The two MATLAB Function blocks only call beacon_supervisor.m and
% beacon_camera.m, so editing those files changes the model without a
% rebuild. Block parameters are variable names from robot_params.m, so
% changing a gain never needs a rebuild either.

here = fileparts(mfilename('fullpath'));
cd(here)
robot_params            % the MATLAB Function blocks need P to exist
build_beacon(here)
disp('Built beacon_approach.slx')

%% Beacon approach model
function build_beacon(here)
m = 'beacon_approach';
if bdIsLoaded(m), close_system(m, 0); end
if isfile(fullfile(here, [m '.slx'])), delete(fullfile(here, [m '.slx'])); end
new_system(m);
set_param(m, 'StopTime', 'Stop_Time', 'MaxStep', '1e-3');

% Supervisor: runs every Ts like the Arduino loop
add_block('simulink/Sources/Digital Clock', [m '/Time'], ...
    'SampleTime', 'Ts', 'Position', [30 45 80 75]);
sup = [m '/Supervisor'];
add_block('simulink/User-Defined Functions/MATLAB Function', sup, ...
    'Position', [150 30 330 290]);
set_function(sup, [ ...
    'function [rho_ref, rho_dot_ref, rho_int, phi_ref, phi_dot_ref, phi_int, stage] = ' ...
    'supervisor(t, rho, phi, cam, P)' newline ...
    '% The stage logic of beacon.ino. See beacon_supervisor.m.' newline ...
    '[rho_ref, rho_dot_ref, rho_int, phi_ref, phi_dot_ref, phi_int, stage] = ' ...
    'beacon_supervisor(t, rho, phi, cam, P);' newline]);

% One channel per coordinate, same structure, different variables (Demo 1)
fw = [m '/Forward'];
add_block('built-in/Subsystem', fw, 'Position', [430 30 550 150]);
build_channel(fw, 'Kp_rho', 'Ki_rho', 'RHO_DOT_LIMIT', 'Kp_rho_dot', 'K_rho', 'sigma_rho');
rot = [m '/Rotation'];
add_block('built-in/Subsystem', rot, 'Position', [430 190 550 310]);
build_channel(rot, 'Kp_phi', 'Ki_phi', 'PHI_DOT_LIMIT', 'Kp_phi_dot', 'K_phi', 'sigma_phi');

% Supervisor outputs 1-3 -> Forward (Ref, RefDot, IntOn), 4-6 -> Rotation
% (the channel's inputs are in the order Ref, RefDot, IntOn)
add_line(m, 'Time/1', 'Supervisor/1', 'autorouting', 'smart');
for k = 1:3
    add_line(m, sprintf('Supervisor/%d', k), sprintf('Forward/%d', k), 'autorouting', 'smart');
    add_line(m, sprintf('Supervisor/%d', k + 3), sprintf('Rotation/%d', k), 'autorouting', 'smart');
end

% (phi, rho_dot) to (x_dot, y_dot), then integrate for the path
add_block('simulink/Signal Routing/Mux', [m '/Mux'], 'Inputs', '2', ...
    'Position', [620 150 625 200]);
add_block('simulink/User-Defined Functions/Fcn', [m '/x_dot'], ...
    'Expr', 'u(1)*cos(u(2))', 'Position', [670 130 760 160]);
add_block('simulink/User-Defined Functions/Fcn', [m '/y_dot'], ...
    'Expr', 'u(1)*sin(u(2))', 'Position', [670 190 760 220]);
add_block('simulink/Continuous/Integrator', [m '/Integrate x'], ...
    'Position', [800 130 830 160]);
add_block('simulink/Continuous/Integrator', [m '/Integrate y'], ...
    'Position', [800 190 830 220]);
add_line(m, 'Forward/2', 'Mux/1', 'autorouting', 'smart');
add_line(m, 'Rotation/1', 'Mux/2', 'autorouting', 'smart');
add_line(m, 'Mux/1', 'x_dot/1', 'autorouting', 'smart');
add_line(m, 'Mux/1', 'y_dot/1', 'autorouting', 'smart');
add_line(m, 'x_dot/1', 'Integrate x/1', 'autorouting', 'smart');
add_line(m, 'y_dot/1', 'Integrate y/1', 'autorouting', 'smart');

% Camera, then the delay before its reading reaches the Arduino
cam = [m '/Camera'];
add_block('simulink/User-Defined Functions/MATLAB Function', cam, ...
    'Position', [430 400 550 500]);
set_function(cam, [ ...
    'function cam = camera(x, y, phi, P)' newline ...
    '% The camera and Pi. See beacon_camera.m.' newline ...
    'cam = beacon_camera(x, y, phi, P);' newline]);
add_block('simulink/Discrete/Delay', [m '/Latency'], ...
    'DelayLength', 'max(1, round(CAM_LATENCY_S/Ts))', 'InitialCondition', '0', ...
    'SampleTime', 'Ts', 'Position', [300 435 350 465], 'Orientation', 'left');
add_line(m, 'Integrate x/1', 'Camera/1', 'autorouting', 'smart');
add_line(m, 'Integrate y/1', 'Camera/2', 'autorouting', 'smart');
add_line(m, 'Rotation/1', 'Camera/3', 'autorouting', 'smart');
add_line(m, 'Camera/1', 'Latency/1', 'autorouting', 'smart');
add_line(m, 'Latency/1', 'Supervisor/4', 'autorouting', 'smart');

% Odometry back to the Supervisor (in the model the encoders are perfect)
add_line(m, 'Forward/1', 'Supervisor/2', 'autorouting', 'smart');
add_line(m, 'Rotation/1', 'Supervisor/3', 'autorouting', 'smart');

% Stop the simulation when the run is done (stage 5 = DONE)
add_block('simulink/Logic and Bit Operations/Compare To Constant', [m '/Done'], ...
    'relop', '>=', 'const', '5', 'Position', [430 340 470 370]);
add_block('simulink/Sinks/Stop Simulation', [m '/Stop'], 'Position', [520 338 555 372]);
add_line(m, 'Supervisor/7', 'Done/1', 'autorouting', 'smart');
add_line(m, 'Done/1', 'Stop/1', 'autorouting', 'smart');

% Outputs
names = {'Rho', 'Vbar', 'X', 'Y', 'Phi', 'DVa', 'RhoRef', 'PhiRef', 'Stage', 'Cam'};
srcs  = {'Forward/1', 'Forward/3', 'Integrate x/1', 'Integrate y/1', 'Rotation/1', ...
         'Rotation/3', 'Supervisor/1', 'Supervisor/4', 'Supervisor/7', 'Latency/1'};
for k = 1:numel(names)
    add_toworkspace(m, names{k}, [920 30 + 50*(k - 1) 990 60 + 50*(k - 1)]);
    add_line(m, srcs{k}, [names{k} '/1'], 'autorouting', 'smart');
end

save_system(m, fullfile(here, [m '.slx']));
close_system(m, 0);
end

%% Put code in a MATLAB Function block, run it every Ts, and make its last
% argument P a parameter (read from the workspace) instead of an input port
function set_function(block, code)
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', block);
chart.Script = code;
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = 'Ts';
p = find(chart, '-isa', 'Stateflow.Data', 'Name', 'P');
p.Scope = 'Parameter';
p.Tunable = false;      % a struct of constants, fixed for each simulation
end

%% One coordinate (rho or phi): outer PI position loop, inner speed loop
% Inputs: 1 reference position, 2 reference speed, 3 integral on (0/1)
% Outputs: 1 position, 2 speed, 3 voltage (Vbar or dVa)
%
%   speed_d = ref_speed + Kp*e + Ki*integral(e*IntOn)    limited to +/-Limit
%   voltage = speed_d/K + Kp_inner*(speed_d - speed)     limited to +/-2*MAX_VOLTAGE
%
% The sketch limits each wheel to MAX_VOLTAGE and scales both together;
% limiting the sum and difference to 2*MAX_VOLTAGE is the closest
% per-channel version. It also undoes integration on saturation, which this
% doesn't model; with the profile limits the voltages stay well below that.
function build_channel(c, Kp, Ki, Limit, Kp_inner, K, sigma)
add_block('simulink/Sources/In1', [c '/Ref'], 'Port', '1', 'Position', [20 40 50 54]);
add_block('simulink/Sources/In1', [c '/RefDot'], 'Port', '2', 'Position', [20 10 50 24]);
add_block('simulink/Sources/In1', [c '/IntOn'], 'Port', '3', 'Position', [20 120 50 134]);

add_block('simulink/Math Operations/Sum', [c '/Error'], ...
    'Inputs', '|+-', 'IconShape', 'round', 'Position', [90 37 110 57]);
add_block('simulink/Math Operations/Gain', [c '/Kp'], 'Gain', Kp, ...
    'Position', [150 32 190 62]);
add_block('simulink/Math Operations/Product', [c '/Gate'], ...
    'Position', [150 100 180 130]);
add_block('simulink/Continuous/Integrator', [c '/Error Integral'], ...
    'Position', [210 100 240 130]);
add_block('simulink/Math Operations/Gain', [c '/Ki'], 'Gain', Ki, ...
    'Position', [270 100 310 130]);
add_block('simulink/Math Operations/Sum', [c '/Speed Command'], ...
    'Inputs', '+++', 'Position', [340 20 360 80]);
add_block('simulink/Discontinuities/Saturation', [c '/Speed Limit'], ...
    'UpperLimit', Limit, 'LowerLimit', ['-' Limit], ...
    'Position', [390 35 430 65]);

add_block('simulink/Math Operations/Gain', [c '/Feedforward'], ...
    'Gain', ['1/' K], 'Position', [470 100 510 130]);
add_block('simulink/Math Operations/Sum', [c '/Speed Error'], ...
    'Inputs', '|+-', 'IconShape', 'round', 'Position', [470 40 490 60]);
add_block('simulink/Math Operations/Gain', [c '/Kp_inner'], 'Gain', Kp_inner, ...
    'Position', [520 35 560 65]);
add_block('simulink/Math Operations/Sum', [c '/Voltage Sum'], ...
    'Inputs', '++', 'Position', [590 20 610 70]);
add_block('simulink/Discontinuities/Saturation', [c '/Voltage Limit'], ...
    'UpperLimit', '2*MAX_VOLTAGE', 'LowerLimit', '-2*MAX_VOLTAGE', ...
    'Position', [640 30 680 60]);
add_block('simulink/Continuous/Transfer Fcn', [c '/Robot'], ...
    'Numerator', [K '*' sigma], 'Denominator', ['[1 ' sigma ']'], ...
    'Position', [710 25 790 65]);
add_block('simulink/Continuous/Integrator', [c '/Position'], ...
    'Position', [830 30 860 60]);

add_block('simulink/Sinks/Out1', [c '/Pos'], 'Port', '1', 'Position', [910 38 940 52]);
add_block('simulink/Sinks/Out1', [c '/Speed'], 'Port', '2', 'Position', [910 98 940 112]);
add_block('simulink/Sinks/Out1', [c '/Voltage'], 'Port', '3', 'Position', [910 158 940 172]);

add_line(c, 'Ref/1', 'Error/1', 'autorouting', 'smart');
add_line(c, 'Position/1', 'Error/2', 'autorouting', 'smart');
add_line(c, 'Error/1', 'Kp/1', 'autorouting', 'smart');
add_line(c, 'Error/1', 'Gate/1', 'autorouting', 'smart');
add_line(c, 'IntOn/1', 'Gate/2', 'autorouting', 'smart');
add_line(c, 'Gate/1', 'Error Integral/1', 'autorouting', 'smart');
add_line(c, 'Error Integral/1', 'Ki/1', 'autorouting', 'smart');
add_line(c, 'RefDot/1', 'Speed Command/1', 'autorouting', 'smart');
add_line(c, 'Kp/1', 'Speed Command/2', 'autorouting', 'smart');
add_line(c, 'Ki/1', 'Speed Command/3', 'autorouting', 'smart');
add_line(c, 'Speed Command/1', 'Speed Limit/1', 'autorouting', 'smart');
add_line(c, 'Speed Limit/1', 'Feedforward/1', 'autorouting', 'smart');
add_line(c, 'Speed Limit/1', 'Speed Error/1', 'autorouting', 'smart');
add_line(c, 'Robot/1', 'Speed Error/2', 'autorouting', 'smart');
add_line(c, 'Speed Error/1', 'Kp_inner/1', 'autorouting', 'smart');
add_line(c, 'Feedforward/1', 'Voltage Sum/1', 'autorouting', 'smart');
add_line(c, 'Kp_inner/1', 'Voltage Sum/2', 'autorouting', 'smart');
add_line(c, 'Voltage Sum/1', 'Voltage Limit/1', 'autorouting', 'smart');
add_line(c, 'Voltage Limit/1', 'Robot/1', 'autorouting', 'smart');
add_line(c, 'Robot/1', 'Position/1', 'autorouting', 'smart');
add_line(c, 'Position/1', 'Pos/1', 'autorouting', 'smart');
add_line(c, 'Robot/1', 'Speed/1', 'autorouting', 'smart');
add_line(c, 'Voltage Limit/1', 'Voltage/1', 'autorouting', 'smart');
end

%% To Workspace block saving a timeseries named like the block
function add_toworkspace(m, name, pos)
add_block('simulink/Sinks/To Workspace', [m '/' name], ...
    'VariableName', name, 'SaveFormat', 'Timeseries', 'Position', pos);
end
