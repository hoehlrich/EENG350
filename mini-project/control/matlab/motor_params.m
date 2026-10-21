%% motor_params.m
% Parameters shared by every control script and Simulink model in this
% folder. Run it (the run_*.m scripts do this for you) before simulating
% openloop_sim.slx or position_control.slx by hand.
%
% Keep the controller values in sync with the constants at the top of
% control/position_control/position_control.ino.

%% Motor models: velocity(s)/voltage(s) = K*sigma/(s + sigma)
% Measure with step_response.ino, readArduinoData, and run_openloop.m,
% which prints fitted values to paste here.
K1     = 2.2;    % left wheel DC gain (rad/s per V)
sigma1 = 5;      % left wheel pole (1/s)
K2     = 2.2;    % right wheel DC gain (rad/s per V)
sigma2 = 5;      % right wheel pole (1/s)

% The models use K and sigma. The run scripts set these per wheel; these
% defaults let the models run straight from the Simulink editor.
K     = K1;
sigma = sigma1;

%% Controller (same names as position_control.ino)
Kp_vel    = 3.2;   % velocity loop gain (V per rad/s), from the working assignment 2 code
Kp_pos    = 5.0;   % position proportional gain ((rad/s) per rad)
Ki_pos    = 2.0;   % position integral gain ((rad/s) per rad*s)
MAX_SPEED = 6.0;   % limit on the desired speed from the position loop (rad/s)

Battery_Voltage = 7.0;   % V, limits the motor voltage
Ts = 0.01;               % Arduino loop time (s)

%% Experiment settings
% Open loop: voltage step from 0 to Step_Voltage at t = 1 s (step_response.ino)
Step_Voltage = 3.0;      % V
Step_Time = 1.0;         % s, time of the voltage step and the first goal change

% Closed loop: position_control.ino with STEP_TEST runs the quadrant table,
% one goal pair after another. Keep in sync with SUITE_GOALS, STEP_TIME and
% STAGE_TIME in the sketch.
Suite_Goals = [0 0       % NE (start)       [left right]
               0 1       % NW: right 0 -> 1
               1 1       % SW: left  0 -> 1
               1 0       % SE: right 1 -> 0
               0 0];     % NE: left  1 -> 0
Stage_Time = 3.0;        % s at each goal pair after the first
Stop_Time = Step_Time + (size(Suite_Goals, 1) - 1) * Stage_Time;   % s, RUN_TIME in the sketch
Step_Position = pi;      % rad, goal 1

% Desired position for position_control.slx ([time, rad] rows). The
% default is the left wheel's part of the suite; run_position.m sets it for
% each wheel.
DesiredPositionIn = goal_signal(Suite_Goals(:, 1), Step_Time, Stage_Time);

% Voltage disturbance added at the motor input, to show integral action.
% Zero for the normal step response; run_position.m sets it for its
% disturbance section.
Disturbance_V    = 0;    % V
Disturbance_Time = 3.0;  % s
