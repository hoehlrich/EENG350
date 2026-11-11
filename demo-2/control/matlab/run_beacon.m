%% run_beacon.m
% Simulation of the Demo 2 run (find the beacon, drive to it, turn the way
% the arrow points), and comparison with a recorded run of beacon.ino.
%
% The model is beacon_approach.slx: the SteeringControl handout's section 6
% diagram with a camera, around the two control channels designed for
% Demo 1 (demo-1/control/matlab/run_motion.m has the loop design: inner
% loop poles and outer loop margins. The gains are unchanged here.)
%
% Experiment: put the beacon where BEACON_HEADING_DEG and
% BEACON_DISTANCE_FT in robot_params.m say, with the arrow ARROW says, and
% the Pi running vision/demo2.py. Upload beacon.ino, put the robot down
% with a USB cable long enough for the run, close the serial monitor, and
% run
%
%     readArduinoData("beacon_data.mat", 12)
%
% Columns: time, stage, rho_ref, rho, phi_ref, phi, VL, VR, cam_angle_deg,
% cam_dist_m, seen_age_s, arrow_votes. Without beacon_data.mat this script
% only plots the simulation.
%
% Publish this script (publish('run_beacon.m', 'pdf')) for a report.

here = fileparts(mfilename('fullpath'));
cd(here)
robot_params
ft = 0.3048;

%% One trial: simulation vs experiment
dataFile = fullfile(here, 'beacon_data.mat');
haveData = isfile(dataFile);
if haveData
    load(dataFile, 'data')
else
    disp('No beacon_data.mat yet - plotting the simulation only.')
end

out = simulateRun(P);
beacon = BEACON_DISTANCE_FT * [cosd(BEACON_HEADING_DEG) sind(BEACON_HEADING_DEG)];   % ft

figure
% Path of the center of rotation. The experiment's path is rebuilt from
% rho and phi: each step moves d(rho) in the direction phi.
subplot(2, 2, 1)
plot(out.X.Data/ft, out.Y.Data/ft, 'linewidth', 2)
hold on
labels = "Simulation";
if haveData
    drho = [0; diff(data(:,4))];
    plot(cumsum(drho .* cos(data(:,6)))/ft, cumsum(drho .* sin(data(:,6)))/ft, ...
         '--', 'linewidth', 2)
    labels(end+1) = "Experiment";
end
plot(beacon(1), beacon(2), 'kx', 'markersize', 12, 'linewidth', 2)
circle = [cosd(0:5:360)' sind(0:5:360)'];
plot(beacon(1) + 1.5*circle(:,1), beacon(2) + 1.5*circle(:,2), 'k:')
axis equal
grid on
xlabel('x (ft)')
ylabel('y (ft)')
title('Path')
legend([labels "Beacon" "1.5 ft"], 'Location', 'best')

subplot(2, 2, 2)
plot(out.PhiRef.Time, out.PhiRef.Data*180/pi, 'k:', 'linewidth', 1.5)
hold on
plot(out.Phi.Time, out.Phi.Data*180/pi, 'linewidth', 2)
if haveData
    plot(data(:,1), data(:,6)*180/pi, '--', 'linewidth', 2)
    legend('Reference', 'Simulation', 'Experiment', 'Location', 'best')
else
    legend('Reference', 'Simulation', 'Location', 'best')
end
xlabel('Time (s)')
ylabel('\phi (deg)')
title('Heading')

subplot(2, 2, 3)
plot(out.RhoRef.Time, out.RhoRef.Data/ft, 'k:', 'linewidth', 1.5)
hold on
plot(out.Rho.Time, out.Rho.Data/ft, 'linewidth', 2)
if haveData
    plot(data(:,1), data(:,4)/ft, '--', 'linewidth', 2)
    legend('Reference', 'Simulation', 'Experiment', 'Location', 'northwest')
else
    legend('Reference', 'Simulation', 'Location', 'northwest')
end
xlabel('Time (s)')
ylabel('\rho (ft)')
title('Distance')

% Stage: 0 wait, 1 search, 2 face, 3 approach, 4 turn, 5 done
subplot(2, 2, 4)
stairs(out.Stage.Time, out.Stage.Data, 'linewidth', 2)
hold on
if haveData
    stairs(data(:,1), data(:,2), '--', 'linewidth', 2)
    legend('Simulation', 'Experiment', 'Location', 'northwest')
end
yticks(0:5)
yticklabels({'wait', 'search', 'face', 'approach', 'turn', 'done'})
ylim([-0.5 5.5])
xlabel('Time (s)')
title('Stage')

r = runResult(out, P);
fprintf('Simulated beacon at %g deg, %g ft: arrived %.1f s, done %.1f s, stopped %.2f ft from it, beacon at %+.0f deg\n', ...
        BEACON_HEADING_DEG, BEACON_DISTANCE_FT, r.arrive, r.done, r.stop_ft, r.bearing_deg)

%% Demo cases
% Start headings all the way round, at the nearest and farthest the beacon
% can be. "Beacon at" is where the beacon ends up as seen from the robot:
% -90 deg (on its right) after a left turn, +90 after a right turn, 0 if it
% only stops (trial type 1). The model has no friction, slip or camera
% noise, so this checks the logic and the timing, not the robot.
fprintf('\n heading  distance  arrow | arrive   done | stop at  beacon at\n')
fprintf('   (deg)      (ft)        |    (s)    (s) |    (ft)      (deg)\n')
for distance = [4 6]
    for heading = [0 45 90 180 -90 -45]
        Pc = P;
        Pc.BEACON_HEADING_DEG = heading;
        Pc.BEACON_DISTANCE_FT = distance;
        Pc.ARROW = 1 + (heading < 0);       % left arrows on one side, right on the other
        r = runResult(simulateRun(Pc), Pc);
        arrows = ["left" "right"];
        fprintf('%8g  %8g  %5s | %6.1f %6.1f | %7.2f  %9.0f\n', heading, distance, ...
                arrows(Pc.ARROW), r.arrive, r.done, r.stop_ft, r.bearing_deg)
    end
end

%% Helper functions
function out = simulateRun(P)
% Simulates beacon_approach.slx for the trial and constants in P. Every
% other parameter comes from robot_params.m in the base workspace.
in = Simulink.SimulationInput('beacon_approach');
in = in.setVariable('P', P);
out = sim(in);
end

function r = runResult(out, P)
% When the robot arrived and finished, and where it ended up
range = P.BEACON_DISTANCE_FT * 0.3048;
bx = range * cosd(P.BEACON_HEADING_DEG);
by = range * sind(P.BEACON_HEADING_DEG);
x = out.X.Data(end);
y = out.Y.Data(end);
after_approach = out.Stage.Time(out.Stage.Data > 3);    % turn or done
if isempty(after_approach)
    r.arrive = NaN;         % never got there
else
    r.arrive = after_approach(1);
end
r.done = out.Stage.Time(end);
r.stop_ft = hypot(bx - x, by - y) / 0.3048;
bearing = atan2(by - y, bx - x) - out.Phi.Data(end);
r.bearing_deg = atan2(sin(bearing), cos(bearing)) * 180 / pi;
end
