function cam = beacon_camera(x, y, phi, P)
%BEACON_CAMERA The camera and Pi, as the Camera block of beacon_approach.slx.
%
%   The handout's camera block (SteeringControl section 6), with what the
%   real one adds: the camera sits ahead of the center of rotation, it has
%   a field of view and a range, and it only takes a picture every
%   CAM_PERIOD_S. Called every Ts seconds; between pictures it repeats the
%   last one. The model delays the output by CAM_LATENCY_S after this.
%
%   Inputs
%     x, y, phi   center of rotation (m) and heading (rad)
%     P           the constants, from robot_params.m
%   Output
%     cam         what vision/demo2.py would send for the newest picture:
%                 [frame number, visible, angle (deg), distance (m), arrow]
%                 angle is from the camera axis, + = marker to the left
%
%   No noise and no missed detections here. tests/host_sim has both.
%#codegen

persistent frame held calls
if isempty(frame)
    frame = 0;
    held = zeros(5, 1);
    calls = 0;
end

frame_samples = max(1, round(P.CAM_PERIOD_S / P.Ts));
if mod(calls, frame_samples) == 0
    % The beacon stands where robot_params.m put it
    range = P.BEACON_DISTANCE_FT * 0.3048;
    beacon_x = range * cos(P.BEACON_HEADING_DEG * pi / 180);
    beacon_y = range * sin(P.BEACON_HEADING_DEG * pi / 180);

    % From the camera lens, in the robot's axes (ahead, left). With no
    % camera offset this is the handout's phi_s = atan2(ys-y, xs-x) - phi.
    dx = beacon_x - (x + P.CAMERA_OFFSET_M * cos(phi) - P.CAMERA_SIDE_M * sin(phi));
    dy = beacon_y - (y + P.CAMERA_OFFSET_M * sin(phi) + P.CAMERA_SIDE_M * cos(phi));
    ahead = dx * cos(phi) + dy * sin(phi);
    left = -dx * sin(phi) + dy * cos(phi);
    angle_deg = atan2(left, ahead) * 180 / pi;
    distance = hypot(ahead, left);

    visible = abs(angle_deg) < P.CAM_FOV_DEG / 2 && distance < P.CAM_RANGE_M;
    frame = frame + 1;
    held = [frame; 0; 0; 0; 0];
    if visible
        arrow = 0;
        if distance < P.CAM_ARROW_RANGE_M, arrow = P.ARROW; end
        held = [frame; 1; angle_deg; distance; arrow];
    end
end
calls = calls + 1;
cam = held;
end
