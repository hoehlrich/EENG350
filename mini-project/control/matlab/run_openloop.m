%% run_openloop.m
% Open loop step response: simulation vs experiment, and a fit of K and
% sigma for each wheel.
%
% Model: openloop_sim.slx, a voltage step of Step_Voltage at t = 1 s into
%
%     velocity(s)/voltage(s) = K*sigma/(s + sigma)
%
% Experiment: step_response.ino applies the same step to both wheels. To
% record it, lift the wheels, upload the sketch, close the serial monitor,
% and run
%
%     readArduinoData("openloop_data.mat", 5)
%
% Columns: time, left voltage, left velocity, right voltage, right velocity.
% Without openloop_data.mat this script only plots the simulation.
%
% Publish this script (publish('run_openloop.m', 'pdf')) for a report.

here = fileparts(mfilename('fullpath'));
cd(here)
motor_params

dataFile = fullfile(here, 'openloop_data.mat');
haveData = isfile(dataFile);
if haveData
    load(dataFile, 'data')
else
    disp('No openloop_data.mat yet - plotting the simulation only.')
end

wheelNames = ["Left", "Right"];
Ks = [K1 K2];
sigmas = [sigma1 sigma2];

%% Simulate each wheel and compare with the experiment
figure
for w = 1:2
    in = Simulink.SimulationInput('openloop_sim');
    in = in.setVariable('K', Ks(w));
    in = in.setVariable('sigma', sigmas(w));
    out = sim(in);

    subplot(2, 2, w)
    plot(out.Voltage.Time, out.Voltage.Data, 'linewidth', 2)
    hold on
    if haveData
        plot(data(:,1), data(:,2*w), '--', 'linewidth', 2)
        legend('Simulation', 'Experiment', 'Location', 'northwest')
    end
    xlabel('Time (s)')
    ylabel('Voltage (V)')
    title(wheelNames(w) + " wheel")

    subplot(2, 2, 2 + w)
    plot(out.Velocity.Time, out.Velocity.Data, 'linewidth', 2)
    hold on
    if haveData
        plot(data(:,1), data(:,2*w + 1), '--', 'linewidth', 2)
        legend('Simulation', 'Experiment', 'Location', 'northwest')
    end
    xlabel('Time (s)')
    ylabel('Angular Velocity (rad/s)')
end

%% Fit K and sigma from the experiment
% K is the steady state velocity divided by the step voltage, using the
% average over the last 0.5 s of the step. The time constant 1/sigma is
% how long the velocity takes to reach 63% of that steady state value.
% The velocity is noisy because of encoder quantization, so the 63%
% crossing is taken as the first sample at or above it.
if haveData
    t = data(:,1);
    for w = 1:2
        V = data(:,2*w);
        vel = data(:,2*w + 1);
        stepOn = V > 0;
        tStep = t(find(stepOn, 1));
        tEnd = t(find(stepOn, 1, 'last'));

        steady = mean(vel(stepOn & t >= tEnd - 0.5));
        Kfit = steady / mean(V(stepOn));

        rise = find(stepOn & vel >= 0.63*steady, 1);
        sigmaFit = 1 / (t(rise) - tStep);

        fprintf('%s wheel: K = %.3f rad/s per V, sigma = %.2f 1/s\n', ...
                wheelNames(w), Kfit, sigmaFit)
    end
    disp('Copy these into motor_params.m and run this script again to check the fit.')
end
