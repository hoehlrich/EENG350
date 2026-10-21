%% run_motion.m
% Controller design and simulation for the Demo 1 turn and drive, and
% comparison with a recorded run of drive.ino.
%
% The controller (SteeringControl handout sections 4 and 5, drive.ino):
%   inner:  V = speed_d/K + Kp_inner*(speed_d - speed)          K*sigma/(s + sigma) plant
%   outer:  speed_d = ref_speed + Kp*e + Ki*integral(e)         on rho and phi
% with the references from the trapezoidal profile in motion_reference.m.
%
% Experiment: set TARGET_ANGLE_DEG and TARGET_DISTANCE_FT the same here
% (robot_params.m) and in drive.ino, upload, put the robot down with a
% USB cable long enough for the run, close the serial monitor, and run
%
%     readArduinoData("drive_data.mat", 7)
%
% Columns: time, rho_ref, rho, phi_ref, phi, VL, VR. Without
% drive_data.mat this script only plots the simulation.
%
% Needs the Control System Toolbox for the design sections.
% Publish this script (publish('run_motion.m', 'pdf')) for a report.

here = fileparts(mfilename('fullpath'));
cd(here)
robot_params

s = tf('s');
G_rho = K_rho*sigma_rho / (s + sigma_rho);
G_phi = K_phi*sigma_phi / (s + sigma_phi);

%% Inner loops: speed step response for several proportional gains
% Closed loop speed/speed_d = (1/K + Kp)*G/(1 + Kp*G). With the 1/K
% feedforward the DC gain is 1 for any Kp; a bigger Kp makes it faster and
% rejects disturbances better, at the cost of more voltage and noise.
figure
scales = [0.25 0.5 1 2];
names = ["rho", "phi"];
Gs = {G_rho, G_phi};
Ks = [K_rho K_phi];
Kps = [Kp_rho_dot Kp_phi_dot];
units = ["V per m/s", "V per rad/s"];
for k = 1:2
    subplot(1, 2, k)
    hold on
    labels = strings(size(scales));
    for j = 1:numel(scales)
        Kp = scales(j) * Kps(k);
        T_inner = minreal((1/Ks(k) + Kp) * Gs{k} / (1 + Kp * Gs{k}));
        step(T_inner, 1)
        labels(j) = sprintf("Kp = %.3g %s", Kp, units(k));
    end
    legend(labels, 'Location', 'southeast')
    title("Inner " + names(k) + "\_dot loop, unit step")
    p = pole(minreal(Kps(k) * Gs{k} / (1 + Kps(k) * Gs{k})));
    fprintf('Inner %s loop with Kp = %.3g: closed loop pole at %.1f 1/s\n', ...
            names(k), Kps(k), p)
end

%% Outer loops: margins on the handout's "yellow box" plant
% G(s) = C_inner*G/((1 + C_inner*G)*s): the closed inner loop followed by
% an integrator from speed to position (handout section 5). The reference
% speed feedforward doesn't change the loop, so the margins are the same
% with or without it.
figure
Kps_outer = [Kp_rho Kp_phi];
Kis_outer = [Ki_rho Ki_phi];
for k = 1:2
    Kp = Kps(k);
    yellow = minreal(Kp * Gs{k} / ((1 + Kp * Gs{k}) * s));
    C = Kps_outer(k) + Kis_outer(k)/s;
    subplot(1, 2, k)
    margin(C * yellow)
    [~, Pm, ~, Wcp] = margin(C * yellow);
    title("Outer " + names(k) + " loop: C(s)G(s)")
    fprintf('Outer %s loop: phase margin %.0f deg at %.2f rad/s\n', names(k), Pm, Wcp)
end

%% The demo run: simulation vs experiment
dataFile = fullfile(here, 'drive_data.mat');
haveData = isfile(dataFile);
if haveData
    load(dataFile, 'data')
else
    disp('No drive_data.mat yet - plotting the simulation only.')
end

out = simulateRun(TARGET_ANGLE_DEG, TARGET_DISTANCE_FT);
ref = motion_reference(TARGET_ANGLE_DEG*pi/180, TARGET_DISTANCE_FT*0.3048, ...
                       Ts, START_DELAY_S, SETTLE_HOLD_S, ...
                       [MAX_RHO_DOT RHO_ACCEL], [MAX_PHI_DOT PHI_ACCEL]);
ft = 0.3048;

figure
subplot(2, 2, 1)
plot(ref.t, ref.rho/ft, 'k:', 'linewidth', 1.5)
hold on
plot(out.Rho.Time, out.Rho.Data/ft, 'linewidth', 2)
if haveData
    plot(data(:,1), data(:,3)/ft, '--', 'linewidth', 2)
    legend('Reference', 'Simulation', 'Experiment', 'Location', 'northwest')
else
    legend('Reference', 'Simulation', 'Location', 'northwest')
end
xlabel('Time (s)')
ylabel('\rho (ft)')
title('Distance')

subplot(2, 2, 2)
plot(ref.t, ref.phi*180/pi, 'k:', 'linewidth', 1.5)
hold on
plot(out.Phi.Time, out.Phi.Data*180/pi, 'linewidth', 2)
if haveData
    plot(data(:,1), data(:,5)*180/pi, '--', 'linewidth', 2)
    legend('Reference', 'Simulation', 'Experiment', 'Location', 'best')
else
    legend('Reference', 'Simulation', 'Location', 'best')
end
xlabel('Time (s)')
ylabel('\phi (deg)')
title('Heading')

% Wheel voltages: VR = (Vbar + dVa)/2, VL = (Vbar - dVa)/2. The sketch also
% adds V_STATIC, which the frictionless model leaves out.
subplot(2, 2, 3)
dva = interp1(out.DVa.Time, out.DVa.Data, out.Vbar.Time);
plot(out.Vbar.Time, (out.Vbar.Data - dva)/2, 'linewidth', 2)
hold on
plot(out.Vbar.Time, (out.Vbar.Data + dva)/2, 'linewidth', 2)
if haveData
    plot(data(:,1), data(:,6), '--', data(:,1), data(:,7), '--', 'linewidth', 1.5)
    legend('VL sim', 'VR sim', 'VL exp', 'VR exp', 'Location', 'best')
else
    legend('VL sim', 'VR sim', 'Location', 'best')
end
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Wheel voltages')

% Path of the center of rotation. The experiment's path is rebuilt from
% rho and phi: each step moves d(rho) in the direction phi.
subplot(2, 2, 4)
plot(out.X.Data/ft, out.Y.Data/ft, 'linewidth', 2)
hold on
if haveData
    drho = [0; diff(data(:,3))];
    plot(cumsum(drho .* cos(data(:,5)))/ft, cumsum(drho .* sin(data(:,5)))/ft, ...
         '--', 'linewidth', 2)
end
goal = TARGET_DISTANCE_FT * [cosd(TARGET_ANGLE_DEG) sind(TARGET_ANGLE_DEG)];
plot(goal(1), goal(2), 'kx', 'markersize', 12, 'linewidth', 2)
axis equal
grid on
xlabel('x (ft)')
ylabel('y (ft)')
title('Path')
if haveData
    legend('Simulation', 'Experiment', 'Goal', 'Location', 'best')
else
    legend('Simulation', 'Goal', 'Location', 'best')
end

[posErr, angErr] = finalError(out, TARGET_ANGLE_DEG, TARGET_DISTANCE_FT);
fprintf('Simulated %g deg, %g ft: final error %.3f in, %.3f deg, done at %.1f s\n', ...
        TARGET_ANGLE_DEG, TARGET_DISTANCE_FT, posErr, angErr, ref.t(end) - 1)

%% Demo cases
% Final error for a few runs like the ones in the demo. The model has no
% friction or slip, so this checks the controller, not the robot.
cases = [0 1; 0 10; 90 3; -45 5; 180 1; 30 10];
for c = 1:size(cases, 1)
    out = simulateRun(cases(c, 1), cases(c, 2));
    [posErr, angErr] = finalError(out, cases(c, 1), cases(c, 2));
    fprintf('%6.1f deg, %4.1f ft: final error %.3f in, %+.3f deg\n', ...
            cases(c, 1), cases(c, 2), posErr, angErr)
end

%% Helper functions
function out = simulateRun(angle_deg, dist_ft)
% Simulates robot_motion.slx for one run, with the other parameters from
% robot_params.m in the base workspace
p = @(name) evalin('base', name);
ref = motion_reference(angle_deg*pi/180, dist_ft*0.3048, p('Ts'), ...
                       p('START_DELAY_S'), p('SETTLE_HOLD_S'), ...
                       [p('MAX_RHO_DOT') p('RHO_ACCEL')], ...
                       [p('MAX_PHI_DOT') p('PHI_ACCEL')]);
in = Simulink.SimulationInput('robot_motion');
in = in.setVariable('RhoRefIn',    [ref.t ref.rho]);
in = in.setVariable('RhoDotRefIn', [ref.t ref.rho_dot]);
in = in.setVariable('RhoIntOnIn',  [ref.t ref.rho_int]);
in = in.setVariable('PhiRefIn',    [ref.t ref.phi]);
in = in.setVariable('PhiDotRefIn', [ref.t ref.phi_dot]);
in = in.setVariable('PhiIntOnIn',  [ref.t ref.phi_int]);
in = in.setVariable('Stop_Time',   ref.t(end));
out = sim(in);
end

function [posErr_in, angErr_deg] = finalError(out, angle_deg, dist_ft)
% Distance from the goal point (in) and heading error (deg) at the end
goal = dist_ft * 0.3048 * [cosd(angle_deg) sind(angle_deg)];
posErr_in = hypot(out.X.Data(end) - goal(1), out.Y.Data(end) - goal(2)) / 0.0254;
angErr_deg = out.Phi.Data(end)*180/pi - angle_deg;
end
