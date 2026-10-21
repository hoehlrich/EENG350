function sig = goal_signal(goals, Step_Time, Stage_Time)
%GOAL_SIGNAL Desired position rows for position_control.slx.
%
%   sig = goal_signal(goals, Step_Time, Stage_Time)
%
%   goals is one wheel's goal (0 or 1) for each stage of the step test, as
%   in a column of Suite_Goals. The first goal holds from t = 0, the next
%   starts at Step_Time, and each one after that Stage_Time later, the same
%   timing as STEP_TEST in position_control.ino. Returns [time, rad] rows
%   for the From Workspace block, which holds each value until the next row.

goals = goals(:);
t = [0; Step_Time + (0:numel(goals) - 2)' * Stage_Time];
sig = [t, pi * goals];
end
