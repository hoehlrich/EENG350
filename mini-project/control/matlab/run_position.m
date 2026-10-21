%% run_position.m
% Closed loop position control through the quadrant table: simulation vs
% experiment, and a simulation showing why the controller needs integral
% action.
%
% Model: position_control.slx. A PI controller on the position error gives
% the desired speed, limited to +/- MAX_SPEED with clamping anti-windup.
% The velocity loop from assignment 2 (Kp_vel, battery limit, motor model)
% turns that into wheel motion. The desired position follows Suite_Goals
% from motor_params.m: NE (0 0), NW (0 1), SW (1 1), SE (1 0), NE (0 0),
% changing at t = 1 s and then every 3 s. Goal 1 is pi rad. Each change
% moves one wheel while the other holds, so each wheel is tested going
% 0 -> 1 and 1 -> 0.
%
% Experiment: position_control.ino with STEP_TEST defined runs the same
% sequence. To record it, set both wheels to 0, upload, close the serial
% monitor, and run
%
%     readArduinoData("position_data.mat", 7)
%
% Columns: time, left voltage, left position, right voltage, right
% position, left goal, right goal. Without position_data.mat this script
% only plots the simulation.
%
% Tuning: start with Ki_pos = 0 and raise Kp_pos until the steps are fast
% without overshoot, then add Ki_pos until disturbances are removed quickly
% (last section). Put the final gains in motor_params.m and in
% position_control.ino.
%
% Publish this script (publish('run_position.m', 'pdf')) for a report.

here = fileparts(mfilename('fullpath'));
cd(here)
motor_params

dataFile = fullfile(here, 'position_data.mat');
haveData = isfile(dataFile);
if haveData
    load(dataFile, 'data')
else
    disp('No position_data.mat yet - plotting the simulation only.')
end

wheelNames = ["Left", "Right"];
Ks = [K1 K2];
sigmas = [sigma1 sigma2];
nStages = size(Suite_Goals, 1);

%% Quadrant table step test for each wheel
figure
for w = 1:2
    in = Simulink.SimulationInput('position_control');
    in = in.setVariable('K', Ks(w));
    in = in.setVariable('sigma', sigmas(w));
    in = in.setVariable('DesiredPositionIn', ...
                        goal_signal(Suite_Goals(:, w), Step_Time, Stage_Time));
    out = sim(in);

    subplot(2, 2, w)
    plot(out.DesiredPosition.Time, out.DesiredPosition.Data, 'k:', 'linewidth', 1.5)
    hold on
    plot(out.Position.Time, out.Position.Data, 'linewidth', 2)
    if haveData
        plot(data(:,1), data(:,2*w + 1), '--', 'linewidth', 2)
        legend('Desired', 'Simulation', 'Experiment', 'Location', 'best')
    else
        legend('Desired', 'Simulation', 'Location', 'best')
    end
    xlabel('Time (s)')
    ylabel('Position (rad)')
    title(wheelNames(w) + " wheel")

    subplot(2, 2, 2 + w)
    plot(out.Voltage.Time, out.Voltage.Data, 'linewidth', 2)
    hold on
    if haveData
        plot(data(:,1), data(:,2*w), '--', 'linewidth', 2)
        legend('Simulation', 'Experiment')
    end
    xlabel('Time (s)')
    ylabel('Voltage (V)')

    % Numbers for each goal change of this wheel, and how well it holds
    % still while only the other wheel moves
    for s = 2:nStages
        t0 = Step_Time + (s - 2) * Stage_Time;
        t1 = t0 + Stage_Time;
        from = pi * Suite_Goals(s - 1, w);
        to = pi * Suite_Goals(s, w);

        if from == to
            [~, simDrift] = holdDrift(out.Position.Time, out.Position.Data, t0, t1, to);
            msg = sprintf('%s wheel holding %d, %4.1f-%4.1f s: max drift sim %.3f rad', ...
                           wheelNames(w), Suite_Goals(s, w), t0, t1, simDrift);
            if haveData
                [~, expDrift] = holdDrift(data(:,1), data(:,2*w + 1), t0, t1, to);
                msg = [msg sprintf(', experiment %.3f rad', expDrift)];
            end
        else
            [os, ts] = stepMetrics(out.Position.Time, out.Position.Data, t0, t1, from, to);
            msg = sprintf('%s wheel %d -> %d at %4.1f s: sim %.1f%% overshoot, settles in %.2f s', ...
                           wheelNames(w), Suite_Goals(s - 1, w), Suite_Goals(s, w), t0, os, ts);
            if haveData
                [os, ts] = stepMetrics(data(:,1), data(:,2*w + 1), t0, t1, from, to);
                msg = [msg sprintf('; experiment %.1f%%, %.2f s', os, ts)];
            end
        end
        disp(msg)
    end
end

%% Disturbance rejection: P only vs PI
% A single goal 0 -> 1 step at Step_Time, then a constant voltage
% disturbance, like a hand pushing on the wheel, at Disturbance_Time. With
% Ki_pos = 0 the wheel settles away from the goal. With integral action it
% is pulled back to exactly pi.
figure
gains = [0, Ki_pos];
labels = ["P only (Ki\_pos = 0)", sprintf("PI (Ki\\_pos = %g)", Ki_pos)];
plot([0 8], Step_Position*[1 1], 'k:', 'linewidth', 1.5)
hold on
for g = 1:2
    in = Simulink.SimulationInput('position_control');
    in = in.setModelParameter('StopTime', '8');
    in = in.setVariable('DesiredPositionIn', goal_signal([0 1], Step_Time, Stage_Time));
    in = in.setVariable('Ki_pos', gains(g));
    in = in.setVariable('Disturbance_V', -3);
    out = sim(in);
    plot(out.Position.Time, out.Position.Data, 'linewidth', 2)
end
xline(Disturbance_Time, '--', '-3 V disturbance')
legend(["Desired", labels], 'Location', 'southeast')
xlabel('Time (s)')
ylabel('Position (rad)')
title('Disturbance rejection (left wheel model)')

%% Helper functions
function [overshoot, settle] = stepMetrics(t, p, t0, t1, from, to)
% Overshoot (% of the step) and 2% settling time for a goal change at t0,
% using samples up to t1, the next goal change. settle is NaN if the wheel
% never stays within 2% before t1.
k = t >= t0 & t < t1;
tt = t(k);
pp = p(k);
stepSize = to - from;
overshoot = 100 * max(0, max((pp - to) * sign(stepSize))) / abs(stepSize);
lastOut = find(abs(pp - to) > 0.02 * abs(stepSize), 1, 'last');
if isempty(lastOut)
    settle = 0;
elseif lastOut == numel(pp)
    settle = NaN;
else
    settle = tt(lastOut + 1) - t0;
end
end

function [k, drift] = holdDrift(t, p, t0, t1, goal)
% Largest distance from the goal while the wheel should be holding still
k = t >= t0 & t < t1;
drift = max(abs(p(k) - goal));
end
