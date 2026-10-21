%% build_models.m
% Builds the two Simulink models of the Demo 1 control subsystem, so they
% can be rebuilt exactly and the structure is readable in this file. The
% built .slx files are committed too, so this only needs to run after a
% change.
%
%   robot_openloop.slx  Voltage step -> K*sigma/(s + sigma). Used for both
%                       G_rho (Vbar step) and G_phi (dVa step) by setting K,
%                       sigma and Step_Input. Compared with
%                       robot_step_response.ino data in run_identify.m.
%
%   robot_motion.slx    The SteeringControl handout section 6 diagram
%                       without the camera: a Forward channel (rho) and a
%                       Rotation channel (phi), each an outer PI position
%                       loop around an inner speed loop, then
%                       x_dot = rho_dot*cos(phi), y_dot = rho_dot*sin(phi)
%                       for the path. The references come from
%                       motion_reference.m. Compared with drive.ino data in
%                       run_motion.m.
%
% Block parameters are variable names from robot_params.m, so changing a
% gain never needs a rebuild.

here = fileparts(mfilename('fullpath'));
build_openloop(here)
build_motion(here)
disp('Built robot_openloop.slx and robot_motion.slx')

%% Open loop model
function build_openloop(here)
m = 'robot_openloop';
if bdIsLoaded(m), close_system(m, 0); end
if isfile(fullfile(here, [m '.slx'])), delete(fullfile(here, [m '.slx'])); end
new_system(m);
set_param(m, 'StopTime', '3', 'MaxStep', '1e-3');   % fine steps for smooth plots

add_block('simulink/Sources/Step', [m '/Voltage Step'], ...
    'Time', 'Step_Time', 'Before', '0', 'After', 'Step_Input', ...
    'Position', [40 80 70 110]);
add_block('simulink/Continuous/Transfer Fcn', [m '/Robot'], ...
    'Numerator', 'K*sigma', 'Denominator', '[1 sigma]', ...
    'Position', [140 75 230 115]);
add_toworkspace(m, 'Output', [310 80 380 110]);
add_toworkspace(m, 'Input', [310 150 380 180]);

add_line(m, 'Voltage Step/1', 'Robot/1', 'autorouting', 'smart');
add_line(m, 'Robot/1', 'Output/1', 'autorouting', 'smart');
add_line(m, 'Voltage Step/1', 'Input/1', 'autorouting', 'smart');

save_system(m, fullfile(here, [m '.slx']));
close_system(m, 0);
end

%% Turn and drive model
function build_motion(here)
m = 'robot_motion';
if bdIsLoaded(m), close_system(m, 0); end
if isfile(fullfile(here, [m '.slx'])), delete(fullfile(here, [m '.slx'])); end
new_system(m);
set_param(m, 'StopTime', 'Stop_Time', 'MaxStep', '1e-3');

% References: [time, value] rows from motion_reference.m
refs = {'RhoRefIn', 'RhoDotRefIn', 'RhoIntOnIn', 'PhiRefIn', 'PhiDotRefIn', 'PhiIntOnIn'};
for k = 1:numel(refs)
    y = 40 + 50*(k - 1) + 40*(k > 3);
    add_block('simulink/Sources/From Workspace', [m '/' refs{k}], ...
        'VariableName', refs{k}, 'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', ...
        'Position', [20 y 100 y+30]);
end

% One channel per coordinate, same structure, different variables
fw = [m '/Forward'];
add_block('built-in/Subsystem', fw, 'Position', [200 40 320 160]);
build_channel(fw, 'Kp_rho', 'Ki_rho', 'RHO_DOT_LIMIT', 'Kp_rho_dot', 'K_rho', 'sigma_rho');
rot = [m '/Rotation'];
add_block('built-in/Subsystem', rot, 'Position', [200 230 320 350]);
build_channel(rot, 'Kp_phi', 'Ki_phi', 'PHI_DOT_LIMIT', 'Kp_phi_dot', 'K_phi', 'sigma_phi');

for k = 1:3
    add_line(m, [refs{k} '/1'], sprintf('Forward/%d', k), 'autorouting', 'smart');
    add_line(m, [refs{k+3} '/1'], sprintf('Rotation/%d', k), 'autorouting', 'smart');
end

% (phi, rho_dot) to (x_dot, y_dot), then integrate for the path
add_block('simulink/Signal Routing/Mux', [m '/Mux'], 'Inputs', '2', ...
    'Position', [420 180 425 230]);
add_block('simulink/User-Defined Functions/Fcn', [m '/x_dot'], ...
    'Expr', 'u(1)*cos(u(2))', 'Position', [470 160 560 190]);
add_block('simulink/User-Defined Functions/Fcn', [m '/y_dot'], ...
    'Expr', 'u(1)*sin(u(2))', 'Position', [470 220 560 250]);
add_block('simulink/Continuous/Integrator', [m '/Integrate x'], ...
    'Position', [600 160 630 190]);
add_block('simulink/Continuous/Integrator', [m '/Integrate y'], ...
    'Position', [600 220 630 250]);

add_line(m, 'Forward/2', 'Mux/1', 'autorouting', 'smart');
add_line(m, 'Rotation/1', 'Mux/2', 'autorouting', 'smart');
add_line(m, 'Mux/1', 'x_dot/1', 'autorouting', 'smart');
add_line(m, 'Mux/1', 'y_dot/1', 'autorouting', 'smart');
add_line(m, 'x_dot/1', 'Integrate x/1', 'autorouting', 'smart');
add_line(m, 'y_dot/1', 'Integrate y/1', 'autorouting', 'smart');

% Outputs
add_toworkspace(m, 'Rho', [700 40 760 70]);
add_toworkspace(m, 'Vbar', [700 100 760 130]);
add_toworkspace(m, 'X', [700 160 760 190]);
add_toworkspace(m, 'Y', [700 220 760 250]);
add_toworkspace(m, 'Phi', [700 280 760 310]);
add_toworkspace(m, 'DVa', [700 340 760 370]);
add_line(m, 'Forward/1', 'Rho/1', 'autorouting', 'smart');
add_line(m, 'Forward/3', 'Vbar/1', 'autorouting', 'smart');
add_line(m, 'Integrate x/1', 'X/1', 'autorouting', 'smart');
add_line(m, 'Integrate y/1', 'Y/1', 'autorouting', 'smart');
add_line(m, 'Rotation/1', 'Phi/1', 'autorouting', 'smart');
add_line(m, 'Rotation/3', 'DVa/1', 'autorouting', 'smart');

save_system(m, fullfile(here, [m '.slx']));
close_system(m, 0);
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
