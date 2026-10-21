%% robot_params.m
% Parameters shared by every control script and Simulink model in this
% folder. Run it (the run_*.m scripts do this for you) before simulating
% robot_openloop.slx or robot_motion.slx by hand.
%
% Names match the constants at the top of control/drive/drive.ino. Keep
% the two in sync.

%% The run (drive.ino: TARGET_ANGLE_DEG, TARGET_DISTANCE_FT, START_DELAY_S)
TARGET_ANGLE_DEG   = 90;     % turn first, positive = left (CCW)
TARGET_DISTANCE_FT = 3;      % then drive this far
START_DELAY_S      = 9.0;    % s

%% Robot geometry - MEASURE with tests/odometry_check (README, "Calibration")
WHEEL_RADIUS_M = 0.0725;     % r
WHEEL_BASE_M   = 0.36;       % d

%% Models (SteeringControl handout section 2)
%   G_rho(s) = rho_dot/Vbar = K_rho*sigma_rho/(s + sigma_rho)   Vbar = VR + VL
%   G_phi(s) = phi_dot/dVa  = K_phi*sigma_phi/(s + sigma_phi)   dVa  = VR - VL
% Fit with robot_step_response.ino and run_identify.m, which prints values
% to paste here and into drive.ino (K_rho, K_phi). Until then they come
% from the mini project's wheel model, theta_dot/V = K*sigma/(s + sigma):
% with VL = VR = Vbar/2, rho_dot = r*K*Vbar/2; with VR = -VL = dVa/2,
% phi_dot = r*K*dVa/d.
K_wheel     = 2.2;   % rad/s per V, mini project motor_params.m
sigma_wheel = 5;     % 1/s
K_rho     = WHEEL_RADIUS_M * K_wheel / 2;              % (m/s) per V
sigma_rho = sigma_wheel;                               % 1/s
K_phi     = WHEEL_RADIUS_M * K_wheel / WHEEL_BASE_M;   % (rad/s) per V
sigma_phi = sigma_wheel;                               % 1/s

%% Controller (drive.ino)
% Inner loops: V = speed_d/K + Kp_*_dot*(speed_d - speed). The proportional
% gains put the inner closed loop pole at sigma*(1 + Kp*K), about 25 1/s
% with the default model. run_motion.m compares other values.
Kp_rho_dot = 48.0;   % V per m/s
Kp_phi_dot = 8.5;    % V per rad/s

% Outer loops: speed_d = ref_speed + Kp*e + Ki*integral(e), the integral
% only after that profile has reached its target
Kp_rho = 3.0;        % (m/s) per m
Ki_rho = 2.0;        % (m/s) per m*s
Kp_phi = 4.0;        % (rad/s) per rad
Ki_phi = 2.0;        % (rad/s) per rad*s
RHO_DOT_LIMIT = 0.15; % m/s
PHI_DOT_LIMIT = 0.75; % rad/s

% Reference profile, slow for testing
MAX_RHO_DOT = 0.1;   % m/s
RHO_ACCEL   = 0.1;   % m/s^2
MAX_PHI_DOT = 0.5;   % rad/s
PHI_ACCEL   = 1.0;   % rad/s^2

% Hard cap on each wheel's voltage
MAX_VOLTAGE = 3.0;   % V

% Stopping
RHO_TOL = 0.003;         % m
PHI_TOL = 0.003;         % rad
SETTLE_HOLD_S = 0.3;     % s

% Static friction. The sketch adds V_STATIC to each wheel voltage; the
% model has no friction, so the simulation leaves it out.
V_STATIC = 0.5;          % V

Battery_Voltage = 7.0;   % V
Ts = 0.01;               % Arduino loop time (s)

%% Identification experiment (robot_step_response.ino)
STEP_VBAR = 2.5;         % V, forward test
STEP_DVA  = 2.5;         % V, rotation test
Step_Time = 1.0;         % s

%% Defaults so the models run straight from the Simulink editor. The run
% scripts overwrite these.
K = K_rho;  sigma = sigma_rho;  Step_Input = STEP_VBAR;
ref = motion_reference(TARGET_ANGLE_DEG*pi/180, TARGET_DISTANCE_FT*0.3048, ...
                       Ts, START_DELAY_S, SETTLE_HOLD_S, ...
                       [MAX_RHO_DOT RHO_ACCEL], [MAX_PHI_DOT PHI_ACCEL]);
RhoRefIn    = [ref.t ref.rho];
RhoDotRefIn = [ref.t ref.rho_dot];
RhoIntOnIn  = [ref.t ref.rho_int];
PhiRefIn    = [ref.t ref.phi];
PhiDotRefIn = [ref.t ref.phi_dot];
PhiIntOnIn  = [ref.t ref.phi_int];
Stop_Time   = ref.t(end);
