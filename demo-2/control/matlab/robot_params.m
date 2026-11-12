%% robot_params.m
% Parameters for the Demo 2 Simulink model (beacon_approach.slx). Run it
% (run_beacon.m does this for you) before simulating the model by hand.
%
% Names match the constants at the top of control/beacon/beacon.ino. Keep
% the two in sync. The robot and controller values are Demo 1's
% (demo-1/control/matlab/robot_params.m), where the loops were designed.

%% The trial (not in the sketch: where the beacon is put for this run)
BEACON_HEADING_DEG = 90;     % beacon direction at the start, from straight ahead, + = left
BEACON_DISTANCE_FT = 5;      % center of rotation to beacon at the start (4 to 6 in the demo)
ARROW              = 1;      % 1 = arrow points left, 2 = right

%% The run (beacon.ino, "the run")
TURN_AT_BEACON   = true;     % true: trial type 2, turn 90 deg at the beacon
STOP_DISTANCE_FT = 1.2;      % stop with the center of rotation this far from the marker
START_DELAY_S    = 1.0;      % s
SEARCH_DIRECTION = 1;        % +1 search turning left, -1 right
DEFAULT_TURN     = 1;        % turn if the arrow was never read

%% Camera as the sketch sees it (beacon.ino, "camera")
CAMERA_OFFSET_M   = 0.03;    % lens ahead of the wheel axis
CAMERA_LATENCY_MS = 250;     % how old the sketch assumes a reading is

%% Camera as it really is (not in the sketch: the model's camera block)
CAM_FOV_DEG      = 60;       % horizontal field of view
CAM_RANGE_M      = 3.0;      % marker too small to detect beyond this
CAM_PERIOD_S     = 0.03;     % time between frames
CAM_LATENCY_S    = 0.25;     % picture taken -> message at the Arduino
CAM_ARROW_RANGE_M = 2.5;     % arrow can't be read beyond this

%% Search and approach (beacon.ino, "search, approach")
SEARCH_PHI_DOT  = 1.5;       % rad/s
SEARCH_STEP_DEG = 45.0;      % 0: turn continuously. > 0: turn this far, pause, repeat
SEARCH_PAUSE_S  = 0.8;       % s, timed from when the robot has stopped
SEARCH_SETTLE_DEG = 2.0;     % deg, counts as stopped within this of the step's heading
MARKER_LOST_S   = 1.5;       % s
FACE_TOL_DEG    = 5.0;       % deg
FACE_RATE       = 0.15;      % rad/s
ARRIVE_TOL_M    = 0.02;      % m
ARRIVE_SPEED    = 0.02;      % m/s
ARROW_MIN_VOTES = 3;
ARROW_MAX_VOTES = 20;
ARROW_WAIT_S    = 2.0;       % s

%% Robot geometry
WHEEL_RADIUS_M = 0.0725;     % r
WHEEL_BASE_M   = 0.35;       % d

%% Models (SteeringControl handout section 2)
%   G_rho(s) = rho_dot/Vbar = K_rho*sigma_rho/(s + sigma_rho)   Vbar = VR + VL
%   G_phi(s) = phi_dot/dVa  = K_phi*sigma_phi/(s + sigma_phi)   dVa  = VR - VL
K_rho     = 0.0825;          % (m/s) per V
sigma_rho = 5;               % 1/s
K_phi     = 0.471;           % (rad/s) per V
sigma_phi = 5;               % 1/s

%% Controller (beacon.ino, "controller")
Kp_rho_dot = 48.0;           % V per m/s
Kp_phi_dot = 8.5;            % V per rad/s
Kp_rho = 3.0;                % (m/s) per m
Ki_rho = 2.0;                % (m/s) per m*s
Kp_phi = 4.0;                % (rad/s) per rad
Ki_phi = 2.0;                % (rad/s) per rad*s
RHO_DOT_LIMIT = 0.45;        % m/s
PHI_DOT_LIMIT = 2.25;        % rad/s

% Reference profile
MAX_RHO_DOT = 0.3;           % m/s
RHO_ACCEL   = 0.3;           % m/s^2
MAX_PHI_DOT = 1.5;           % rad/s
PHI_ACCEL   = 3.0;           % rad/s^2

MAX_VOLTAGE = 4.0;           % V, cap on each wheel
SETTLE_HOLD_S = 0.3;         % s
Ts = 0.01;                   % Arduino loop time (s)

Stop_Time = 120;             % s, the model stops itself when the run is done

%% Everything above in one struct, for the two MATLAB Function blocks
% (beacon_supervisor.m and beacon_camera.m read their constants from P)
P = struct();
for name = ["BEACON_HEADING_DEG" "BEACON_DISTANCE_FT" "ARROW" "TURN_AT_BEACON" ...
            "STOP_DISTANCE_FT" "START_DELAY_S" "SEARCH_DIRECTION" "DEFAULT_TURN" ...
            "CAMERA_OFFSET_M" "CAMERA_LATENCY_MS" "CAM_FOV_DEG" "CAM_RANGE_M" ...
            "CAM_PERIOD_S" "CAM_LATENCY_S" "CAM_ARROW_RANGE_M" "SEARCH_PHI_DOT" ...
            "SEARCH_STEP_DEG" "SEARCH_PAUSE_S" "SEARCH_SETTLE_DEG" "MARKER_LOST_S" "FACE_TOL_DEG" ...
            "FACE_RATE" "ARRIVE_TOL_M" "ARRIVE_SPEED" "ARROW_MIN_VOTES" ...
            "ARROW_MAX_VOTES" "ARROW_WAIT_S" "MAX_RHO_DOT" "RHO_ACCEL" ...
            "MAX_PHI_DOT" "PHI_ACCEL" "SETTLE_HOLD_S" "Ts"]
    P.(name) = double(eval(name));
end
clear name
