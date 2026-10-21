function data = readArduinoData(outFile, nColumns, port, baud)
%READARDUINODATA Record one experiment from the Arduino serial port.
%
%   data = readArduinoData(outFile, nColumns)
%   data = readArduinoData(outFile, nColumns, port, baud)
%
%   Opens the serial port, which resets the Arduino and restarts its
%   sketch, waits for "Ready!", keeps every line with nColumns numbers on
%   it, and stops at "Finished". The rows are returned and saved as the
%   variable data in outFile, next to this function, where run_openloop.m
%   and run_position.m look for them.
%
%   Open loop step (step_response.ino):
%       readArduinoData("openloop_data.mat", 5)
%   Closed loop quadrant table test (position_control.ino with STEP_TEST):
%       readArduinoData("position_data.mat", 7)
%
%   Both sketches print time, then voltage and velocity/position for the
%   left wheel, then the same for the right wheel. position_control.ino
%   also prints the left and right goals.
%
%   Before running: upload the sketch, close the Arduino IDE serial monitor
%   (only one program can hold the port), and set the wheels to 0, because
%   the reset zeroes the encoders. For the open loop test, lift the wheels.
%
%   Defaults: port "/dev/ttyACM0", baud 115200. On Windows the port is
%   something like "COM3" (see serialportlist).

arguments
    outFile  (1,1) string
    nColumns (1,1) double {mustBePositive, mustBeInteger}
    port     (1,1) string = "/dev/ttyACM0"
    baud     (1,1) double = 115200
end

%% Open the serial port
% Creating the serialport object resets the Uno, so the run is caught from
% the beginning. The bootloader takes a couple of seconds.
sp = serialport(port, baud, "Timeout", 10);
cleanup = onCleanup(@() clear("sp"));   % release the port even on error
configureTerminator(sp, "LF");
flush(sp);

%% Wait for the Arduino to announce itself
disp('Waiting for the Arduino...')
line = "";
startWait = tic;
while ~strcmp(line, "Ready!")
    if toc(startWait) > 15
        error(['No "Ready!" received. Check the port, that the serial ' ...
               'monitor is closed, and that the right sketch is loaded.'])
    end
    line = readline(sp);
    if ismissing(line)
        line = "";
    else
        line = strtrim(line);
    end
end

%% Collect the data until "Finished"
disp('Recording...')
data = zeros(0, nColumns);
while true
    line = readline(sp);
    if ismissing(line)
        error('The Arduino stopped sending data before printing "Finished".')
    end

    line = strtrim(line);
    if strcmp(line, "Finished")
        break
    end

    values = sscanf(line, '%f');
    if numel(values) == nColumns
        data(end+1, :) = values';  %#ok<AGROW>
    end
end

%% Save beside this file so the run scripts find it
outPath = fullfile(fileparts(mfilename('fullpath')), outFile);
save(outPath, 'data')
fprintf('Saved %d rows to %s\n', size(data, 1), outPath)
end
