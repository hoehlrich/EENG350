function ref = motion_reference(angle_rad, dist_m, Ts, start_delay, settle_hold, rho_limits, phi_limits)
%MOTION_REFERENCE Reference signals for robot_motion.slx, as drive.ino makes them.
%
%   ref = motion_reference(angle_rad, dist_m, Ts, start_delay, settle_hold,
%                          [MAX_RHO_DOT RHO_ACCEL], [MAX_PHI_DOT PHI_ACCEL])
%
%   Steps the same trapezoidal profile as stepProfile() in drive.ino every
%   Ts seconds: nothing until start_delay, then phi goes to angle_rad, then
%   rho goes to dist_m. On the robot each stage ends when the robot has
%   settled; here it ends settle_hold after the profile finishes, as if the
%   robot tracked perfectly. One more second is added at the end.
%
%   Returns a struct of columns sampled at Ts:
%     t                 time (s)
%     rho, rho_dot      distance reference (m) and its speed (m/s)
%     phi, phi_dot      heading reference (rad) and its speed (rad/s)
%     rho_int, phi_int  1 when that profile is at its target, so the
%                       integral of its error runs (0 otherwise)

stage = "WAIT";
rho = [0 0];  phi = [0 0];        % [position speed]
rho_target = 0;  phi_target = 0;
t = 0;  done_at = NaN;
rows = zeros(0, 7);

while true
    if stage == "WAIT" && t >= start_delay
        stage = "TURN";
        phi_target = angle_rad;
    end
    if stage == "TURN" || stage == "DRIVE"
        rho = stepProfile(rho, rho_target, rho_limits(1), rho_limits(2), Ts);
        phi = stepProfile(phi, phi_target, phi_limits(1), phi_limits(2), Ts);
        rho_done = isequal(rho, [rho_target 0]);
        phi_done = isequal(phi, [phi_target 0]);
        if rho_done && phi_done && isnan(done_at)
            done_at = t;
        end
        if ~isnan(done_at) && t - done_at >= settle_hold
            if stage == "TURN"
                stage = "DRIVE";
                rho_target = dist_m;
                done_at = NaN;
            else
                stage = "DONE";
                t_end = t + 1;
            end
        end
    end
    rows(end+1, :) = [t, rho, phi, ...
                      isequal(rho, [rho_target 0]), isequal(phi, [phi_target 0])]; %#ok<AGROW>
    if stage == "DONE" && t >= t_end
        break
    end
    t = t + Ts;
end

ref.t       = rows(:, 1);
ref.rho     = rows(:, 2);
ref.rho_dot = rows(:, 3);
ref.phi     = rows(:, 4);
ref.phi_dot = rows(:, 5);
ref.rho_int = rows(:, 6);
ref.phi_int = rows(:, 7);
end

function p = stepProfile(p, target, vmax, amax, dt)
% Same as stepProfile() in drive.ino. p = [position speed].
remaining = target - p(1);
if remaining >= 0, dir = 1; else, dir = -1; end
v_want = dir * min(vmax, sqrt(2 * amax * abs(remaining)));
p(2) = p(2) + min(max(v_want - p(2), -amax*dt), amax*dt);
p(1) = p(1) + p(2) * dt;
if (target - p(1)) * dir <= 0
    p = [target 0];
end
end
