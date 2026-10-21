%% run_identify.m
% Identifies G_rho(s) and G_phi(s) from open loop steps of the whole robot
% on the floor (SteeringControl handout section 3), and compares the
% fitted models with the experiment.
%
%   G_rho(s) = rho_dot/Vbar = K_rho*sigma_rho/(s + sigma_rho)
%   G_phi(s) = phi_dot/dVa  = K_phi*sigma_phi/(s + sigma_phi)
%
% Model: robot_openloop.slx, a step of Step_Input at t = 1 s into
% K*sigma/(s + sigma).
%
% Experiment: robot_step_response.ino. Upload it, put the robot on the
% floor with room ahead, close the serial monitor, and run
%
%     readArduinoData("rho_step_data.mat", 5)
%
% then define TEST_ROTATION in the sketch, upload again, and run
%
%     readArduinoData("phi_step_data.mat", 5)
%
% Columns: time, Vbar, dVa, rho_dot, phi_dot. Without the data files this
% script only plots the simulation.
%
% Publish this script (publish('run_identify.m', 'pdf')) for a report.

here = fileparts(mfilename('fullpath'));
cd(here)
robot_params

% One row per experiment: name, data file, input column, output column,
% the other output column, K, sigma, step size, units
tests = struct( ...
    'name',   {"Forward (rho)", "Rotation (phi)"}, ...
    'file',   {"rho_step_data.mat", "phi_step_data.mat"}, ...
    'in',     {2, 3}, ...
    'out',    {4, 5}, ...
    'other',  {5, 4}, ...
    'K',      {K_rho, K_phi}, ...
    'sigma',  {sigma_rho, sigma_phi}, ...
    'step',   {STEP_VBAR, STEP_DVA}, ...
    'inName', {"Vbar (V)", "dVa (V)"}, ...
    'outName',{"rho\_dot (m/s)", "phi\_dot (rad/s)"}, ...
    'otherName', {"phi\_dot (rad/s)", "rho\_dot (m/s)"}, ...
    'var',    {"rho", "phi"});

%% Simulate each step and compare with the experiment
figure
for k = 1:2
    T = tests(k);
    haveData = isfile(fullfile(here, T.file));
    if haveData
        S = load(fullfile(here, T.file), 'data');
        data = S.data;
    else
        disp("No " + T.file + " yet - plotting the simulation only.")
    end

    in = Simulink.SimulationInput('robot_openloop');
    in = in.setVariable('K', T.K);
    in = in.setVariable('sigma', T.sigma);
    in = in.setVariable('Step_Input', T.step);
    out = sim(in);

    % The stepped output
    subplot(2, 2, k)
    plot(out.Output.Time, out.Output.Data, 'linewidth', 2)
    hold on
    if haveData
        plot(data(:,1), data(:,T.out), '--', 'linewidth', 2)
        legend('Simulation', 'Experiment', 'Location', 'northwest')
    end
    xlabel('Time (s)')
    ylabel(T.outName)
    title(T.name + " step")

    % The other output should stay near 0 if the two are decoupled
    subplot(2, 2, 2 + k)
    if haveData
        plot(data(:,1), data(:,T.other), 'linewidth', 2)
    end
    xlabel('Time (s)')
    ylabel(T.otherName)
    title("Coupling into " + T.otherName)
end

%% Fit K and sigma from the experiments
% K is the steady state output divided by the step input, using the
% average over the last 0.5 s of the step. The time constant 1/sigma is how
% long the output takes to reach 63% of that steady state value, taken as
% the first sample at or above it (the speed is noisy from encoder
% quantization). Same method as the mini project's run_openloop.m.
fitted = false;
for k = 1:2
    T = tests(k);
    if ~isfile(fullfile(here, T.file))
        continue
    end
    fitted = true;
    S = load(fullfile(here, T.file), 'data');
    t = S.data(:,1);
    u = S.data(:,T.in);
    y = S.data(:,T.out);

    stepOn = u ~= 0;
    tStep = t(find(stepOn, 1));
    tEnd = t(find(stepOn, 1, 'last'));
    steady = mean(y(stepOn & t >= tEnd - 0.5));
    Kfit = steady / mean(u(stepOn));
    rise = find(stepOn & y >= 0.63*steady, 1);
    sigmaFit = 1 / (t(rise) - tStep);

    fprintf('%s: K_%s = %.4f, sigma_%s = %.2f 1/s\n', ...
            T.name, T.var, Kfit, T.var, sigmaFit)
end
if fitted
    disp('Copy these into robot_params.m (and K_rho, K_phi into drive.ino),')
    disp('then run this script again to check the fit.')
end
