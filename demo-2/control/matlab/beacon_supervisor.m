function [rho_ref, rho_dot_ref, rho_int, phi_ref, phi_dot_ref, phi_int, stage_out] = ...
        beacon_supervisor(t, rho, phi, cam, P)
%BEACON_SUPERVISOR The stage logic and reference profiles of beacon.ino.
%
%   Called every Ts seconds by the Supervisor block of beacon_approach.slx.
%   It is the part of the sketch's loop() above "Controller (Demo 1)":
%   it turns camera readings into a heading target and a distance target,
%   moves through the stages, and steps the two trapezoidal profiles. The
%   loops that follow the references are the model's Forward and Rotation
%   blocks.
%
%   Inputs
%     t         time (s)
%     rho, phi  odometry: distance traveled (m) and heading (rad)
%     cam       newest message from the camera block:
%               [frame number, visible, angle (deg), distance (m), arrow]
%     P         the constants, from robot_params.m
%   Outputs
%     rho_ref, rho_dot_ref   distance reference (m) and its speed (m/s)
%     phi_ref, phi_dot_ref   heading reference (rad) and its speed (rad/s)
%     rho_int, phi_int       1 when that profile is at its target, so the
%                            integral of its error runs
%     stage_out              0 WAIT, 1 SEARCH, 2 FACE, 3 APPROACH, 4 TURN,
%                            5 DONE (the numbers beacon.ino prints)
%
%   Differences from the sketch: the model has no friction or deadband, and
%   in DONE the references are held instead of the motors being switched
%   off, so the model shows no coasting after the last turn.
%#codegen

persistent stage rho_prof phi_prof rho_target phi_target
persistent rho_history phi_history head
persistent marker_phi marker_rho last_seen last_frame votes
persistent arrived_time pause_since

WAIT = 0;  SEARCH = 1;  FACE = 2;  APPROACH = 3;  TURN = 4;  DONE = 5;
HISTORY_LEN = 32;

if isempty(stage)
    stage = WAIT;
    rho_prof = [0 0];  phi_prof = [0 0];      % [position speed]
    rho_target = 0;  phi_target = 0;
    rho_history = zeros(1, HISTORY_LEN);
    phi_history = zeros(1, HISTORY_LEN);
    head = 1;
    marker_phi = 0;  marker_rho = 0;
    last_seen = -1;  last_frame = 0;  votes = 0;
    arrived_time = -1;  pause_since = -1;
end

%% Odometry history, to match a camera reading with where the robot was
head = mod(head, HISTORY_LEN) + 1;
rho_history(head) = rho;
phi_history(head) = phi;

%% Camera: a sighting is a new frame with the marker in it
seen = cam(1) ~= last_frame && cam(2) > 0;
last_frame = cam(1);
if seen
    angle = cam(3) * pi / 180;
    % Marker position seen from the center of rotation: x ahead, y left
    mx = P.CAMERA_OFFSET_M + cam(4) * cos(angle);
    my = P.CAMERA_SIDE_M + cam(4) * sin(angle);
    latency_samples = min(round(P.CAMERA_LATENCY_MS / 1000 / P.Ts), HISTORY_LEN - 1);
    then = mod(head - 1 - latency_samples, HISTORY_LEN) + 1;
    marker_phi = phi_history(then) + atan2(my, mx);
    marker_rho = rho_history(then) + hypot(mx, my) - P.STOP_DISTANCE_FT * 0.3048;
    last_seen = t;
    if cam(5) == 1, votes = min(votes + 1,  P.ARROW_MAX_VOTES); end
    if cam(5) == 2, votes = max(votes - 1, -P.ARROW_MAX_VOTES); end
end

%% Stages: choose the targets
if stage == WAIT && t >= P.START_DELAY_S
    stage = SEARCH;
end

if stage == SEARCH
    if seen
        stage = FACE;
    elseif P.SEARCH_STEP_DEG <= 0
        phi_target = phi_prof(1) + P.SEARCH_DIRECTION * pi;
    elseif isequal(phi_prof, [phi_target 0]) && abs(phi_target - phi) < P.SEARCH_SETTLE_DEG * pi / 180
        % stopped on the step (the sketch also gives up waiting after 1 s)
        if pause_since < 0
            phi_target = phi;  phi_prof = [phi 0];     % hold where it stopped
            pause_since = t;
        elseif t - pause_since >= P.SEARCH_PAUSE_S
            phi_target = phi_target + P.SEARCH_DIRECTION * P.SEARCH_STEP_DEG * pi / 180;
            pause_since = -1;
        end
    end
end

if stage == FACE
    if seen, phi_target = marker_phi; end
    if abs(phi_target - phi) < P.FACE_TOL_DEG * pi / 180 && abs(phi_prof(2)) < P.FACE_RATE
        stage = APPROACH;
        rho_target = marker_rho;
    elseif isequal(phi_prof, [phi_target 0]) && t - last_seen > P.MARKER_LOST_S
        stage = SEARCH;
        pause_since = -1;
    end
end

if stage == APPROACH
    if arrived_time < 0
        if seen
            phi_target = marker_phi;
            rho_target = marker_rho;
        end
        if abs(rho_target - rho) < P.ARRIVE_TOL_M && abs(rho_prof(2)) < P.ARRIVE_SPEED
            arrived_time = t;
        end
    end
    if arrived_time >= 0
        arrow_known = abs(votes) >= P.ARROW_MIN_VOTES;
        if ~P.TURN_AT_BEACON
            stage = DONE;
        elseif arrow_known || t - arrived_time >= P.ARROW_WAIT_S
            turn = P.DEFAULT_TURN;
            if arrow_known, turn = sign(votes); end
            phi_target = phi_target + turn * pi / 2;
            stage = TURN;
        end
    end
end

%% Reference profiles
if stage ~= WAIT && stage ~= DONE
    phi_speed = P.MAX_PHI_DOT;  phi_accel = P.PHI_ACCEL;
    if stage == SEARCH, phi_speed = P.SEARCH_PHI_DOT;  phi_accel = P.SEARCH_PHI_ACCEL; end
    rho_prof = stepProfile(rho_prof, rho_target, P.MAX_RHO_DOT, P.RHO_ACCEL, P.Ts);
    phi_prof = stepProfile(phi_prof, phi_target, phi_speed, phi_accel, P.Ts);

    % The last turn ends as soon as the heading is within TURN_DONE_TOL_DEG
    if stage == TURN && abs(phi_target - phi) < P.TURN_DONE_TOL_DEG * pi / 180
        stage = DONE;
    end
end

rho_ref = rho_prof(1);
rho_dot_ref = rho_prof(2);
rho_int = double(isequal(rho_prof, [rho_target 0]));
phi_ref = phi_prof(1);
phi_dot_ref = phi_prof(2);
phi_int = double(isequal(phi_prof, [phi_target 0]));
stage_out = stage;
end

function p = stepProfile(p, target, vmax, amax, dt)
% Same as stepProfile() in beacon.ino. p = [position speed].
remaining = target - p(1);
if remaining >= 0, dir = 1; else, dir = -1; end
v_want = dir * min(vmax, sqrt(2 * amax * abs(remaining)));
p(2) = p(2) + min(max(v_want - p(2), -amax*dt), amax*dt);
p(1) = p(1) + p(2) * dt;
if (target - p(1)) * dir <= 0 && abs(p(2)) <= 4 * amax * dt
    p = [target 0];
end
end
