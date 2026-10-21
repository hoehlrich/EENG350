%% build_models.m
% Builds the two Simulink models used by the control subsystem, so they can
% be rebuilt exactly and the structure is readable in this file. The built
% .slx files are committed too, so this only needs to run after a change.
%
%   openloop_sim.slx      Voltage step -> motor. Compared with
%                         step_response.ino data in run_openloop.m.
%
%   position_control.slx  PI position loop around the proportional
%                         velocity loop (mini project handout section 6).
%                         The desired position comes from the workspace, so
%                         it can follow the quadrant goal sequence. Compared
%                         with position_control.ino STEP_TEST data in
%                         run_position.m.
%
% Block parameters are variable names (K, sigma, Kp_pos, ...) that come
% from motor_params.m, so changing a gain never needs a rebuild.

here = fileparts(mfilename('fullpath'));
build_openloop(here)
build_position(here)
disp('Built openloop_sim.slx and position_control.slx')

%% Open loop model
function build_openloop(here)
m = 'openloop_sim';
if bdIsLoaded(m), close_system(m, 0); end
if isfile(fullfile(here, [m '.slx'])), delete(fullfile(here, [m '.slx'])); end
new_system(m);
set_param(m, 'StopTime', '3', 'MaxStep', '1e-3');   % fine steps for smooth plots

add_block('simulink/Sources/Step', [m '/Voltage Step'], ...
    'Time', 'Step_Time', 'Before', '0', 'After', 'Step_Voltage', ...
    'Position', [40 80 70 110]);
add_block('simulink/Continuous/Transfer Fcn', [m '/Motor'], ...
    'Numerator', 'K*sigma', 'Denominator', '[1 sigma]', ...
    'Position', [140 75 230 115]);
add_toworkspace(m, 'Velocity', [310 80 380 110]);
add_toworkspace(m, 'Voltage', [310 150 380 180]);

add_line(m, 'Voltage Step/1', 'Motor/1', 'autorouting', 'smart');
add_line(m, 'Motor/1', 'Velocity/1', 'autorouting', 'smart');
add_line(m, 'Voltage Step/1', 'Voltage/1', 'autorouting', 'smart');

save_system(m, fullfile(here, [m '.slx']));
close_system(m, 0);
end

%% Closed loop position model
function build_position(here)
m = 'position_control';
if bdIsLoaded(m), close_system(m, 0); end
if isfile(fullfile(here, [m '.slx'])), delete(fullfile(here, [m '.slx'])); end
new_system(m);
set_param(m, 'StopTime', 'Stop_Time', 'MaxStep', '1e-3');   % fine steps for smooth plots

% Desired position: [time, rad] rows in DesiredPositionIn, each value held
% until the next row. motor_params.m and run_position.m build it from the
% goal sequence (see goal_signal.m).
add_block('simulink/Sources/From Workspace', [m '/Desired Position'], ...
    'VariableName', 'DesiredPositionIn', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', [20 100 70 130]);
add_block('simulink/Math Operations/Sum', [m '/Position Error'], ...
    'Inputs', '|+-', 'IconShape', 'round', 'Position', [110 105 130 125]);

% PI position controller. Output limit and clamping anti-windup match
% MAX_SPEED and the undo-integration step in position_control.ino.
add_block('simulink/Continuous/PID Controller', [m '/Position PI'], ...
    'Controller', 'PI', 'P', 'Kp_pos', 'I', 'Ki_pos', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', 'MAX_SPEED', ...
    'LowerSaturationLimit', '-MAX_SPEED', 'AntiWindupMode', 'clamping', ...
    'Position', [170 95 240 135]);

% Disturbance voltage at the motor input (zero unless run_position.m sets it)
add_block('simulink/Sources/Step', [m '/Disturbance'], ...
    'Time', 'Disturbance_Time', 'Before', '0', 'After', 'Disturbance_V', ...
    'Position', [200 180 230 210]);

% Velocity loop from assignment 2, as a subsystem
vc = [m '/Velocity Control'];
add_block('built-in/Subsystem', vc, 'Position', [290 95 400 185]);
build_velocity_loop(vc);

% Position is the integral of velocity
add_block('simulink/Continuous/Integrator', [m '/Integrator'], ...
    'Position', [460 100 490 130]);

add_toworkspace(m, 'Position', [560 100 630 130]);
add_toworkspace(m, 'Voltage', [560 160 630 190]);
add_toworkspace(m, 'DesiredPosition', [110 30 200 60]);

add_line(m, 'Desired Position/1', 'Position Error/1', 'autorouting', 'smart');
add_line(m, 'Desired Position/1', 'DesiredPosition/1', 'autorouting', 'smart');
add_line(m, 'Position Error/1', 'Position PI/1', 'autorouting', 'smart');
add_line(m, 'Position PI/1', 'Velocity Control/1', 'autorouting', 'smart');
add_line(m, 'Disturbance/1', 'Velocity Control/2', 'autorouting', 'smart');
add_line(m, 'Velocity Control/1', 'Integrator/1', 'autorouting', 'smart');
add_line(m, 'Velocity Control/2', 'Voltage/1', 'autorouting', 'smart');
add_line(m, 'Integrator/1', 'Position/1', 'autorouting', 'smart');
add_line(m, 'Integrator/1', 'Position Error/2', 'autorouting', 'smart');

save_system(m, fullfile(here, [m '.slx']));
close_system(m, 0);
end

%% Velocity loop: Kp_vel on the speed error, battery limit, motor model
function build_velocity_loop(vc)
add_block('simulink/Sources/In1', [vc '/Desired Speed'], 'Port', '1', ...
    'Position', [20 43 50 57]);
add_block('simulink/Sources/In1', [vc '/Disturbance'], 'Port', '2', ...
    'Position', [300 123 330 137]);
add_block('simulink/Math Operations/Sum', [vc '/Speed Error'], ...
    'Inputs', '|+-', 'IconShape', 'round', 'Position', [90 40 110 60]);
add_block('simulink/Math Operations/Gain', [vc '/Kp_vel'], ...
    'Gain', 'Kp_vel', 'Position', [150 35 190 65]);
add_block('simulink/Discontinuities/Saturation', [vc '/Battery Limit'], ...
    'UpperLimit', 'Battery_Voltage', 'LowerLimit', '-Battery_Voltage', ...
    'Position', [230 35 270 65]);
add_block('simulink/Math Operations/Sum', [vc '/Add Disturbance'], ...
    'Inputs', '|++', 'IconShape', 'round', 'Position', [330 40 350 60]);
add_block('simulink/Continuous/Transfer Fcn', [vc '/Motor'], ...
    'Numerator', 'K*sigma', 'Denominator', '[1 sigma]', ...
    'Position', [390 30 470 70]);
add_block('simulink/Sinks/Out1', [vc '/Velocity'], 'Port', '1', ...
    'Position', [530 43 560 57]);
add_block('simulink/Sinks/Out1', [vc '/Voltage'], 'Port', '2', ...
    'Position', [530 173 560 187]);

add_line(vc, 'Desired Speed/1', 'Speed Error/1', 'autorouting', 'smart');
add_line(vc, 'Speed Error/1', 'Kp_vel/1', 'autorouting', 'smart');
add_line(vc, 'Kp_vel/1', 'Battery Limit/1', 'autorouting', 'smart');
add_line(vc, 'Battery Limit/1', 'Add Disturbance/1', 'autorouting', 'smart');
add_line(vc, 'Disturbance/1', 'Add Disturbance/2', 'autorouting', 'smart');
add_line(vc, 'Add Disturbance/1', 'Motor/1', 'autorouting', 'smart');
add_line(vc, 'Motor/1', 'Velocity/1', 'autorouting', 'smart');
add_line(vc, 'Motor/1', 'Speed Error/2', 'autorouting', 'smart');
add_line(vc, 'Battery Limit/1', 'Voltage/1', 'autorouting', 'smart');
end

%% To Workspace block saving a timeseries named like the block
function add_toworkspace(m, name, pos)
add_block('simulink/Sinks/To Workspace', [m '/' name], ...
    'VariableName', name, 'SaveFormat', 'Timeseries', 'Position', pos);
end
