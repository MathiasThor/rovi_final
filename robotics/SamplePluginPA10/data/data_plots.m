%% ROVI final project - Data plotting
% Plotting data from the robot
clc; clear;

% Variables used for loading data
folder = './data/';
delim = '\t';
header = 0;

% Data matrices
joint_import  = importdata('new_points/joint_positions_1.000000.txt', delim, header +1);
tool_import   = importdata('new_points/tool_positions_1.000000.txt', delim, header);


dt = joint_import.textdata;

%%
joint_data = joint_import.data
tool_data = [];

[rows, columns] = size(tool_import);

for row = 1:1:rows
    temp_rot = [tool_import(row, [4:6]); tool_import(row, [7:9]); tool_import(row, [9:11])];
    eul = rotm2eul(temp_rot);
    
    tool_data = [tool_data; tool_import(row, [1:3]), eul];
end

%joint_data    = joint_import.data;
%tool_data     = tool_import.data;

%[samples,obs] = size(joint_data);

dt = 0:1:500;

figure(1)
subplot(2,2,[1,3])
hold on
plot(dt, joint_data(:,1), '--', 'LineWidth', 2, 'DisplayName','Q1');
plot(dt, joint_data(:,2), 'LineWidth', 2, 'DisplayName','Q2');
plot(dt, joint_data(:,3), '--', 'LineWidth', 2, 'DisplayName','Q3');
plot(dt, joint_data(:,4), 'LineWidth', 2, 'DisplayName','Q4');
plot(dt, joint_data(:,5), '--', 'LineWidth', 2, 'DisplayName','Q5');
plot(dt, joint_data(:,6), 'LineWidth', 2, 'DisplayName','Q6');
plot(dt, joint_data(:,7), '--', 'LineWidth', 2, 'DisplayName','Q7');
hold off
title('Joint configurations')
ylabel('Q [radians]')
xlabel('Time [s]')
legend('show')
axis([0 inf -2.5 2])
subplot(2,2,2)
hold on
plot(dt, tool_data(:,1), 'LineWidth', 2 , 'DisplayName','X');
plot(dt, tool_data(:,2), 'LineWidth', 2 ,'DisplayName','Y');
plot(dt, tool_data(:,3), 'LineWidth', 2 ,'DisplayName','Z');
hold off
title('Tool Position')
ylabel('Coordinate [m]');
xlabel('Time [s]')
legend('show')
axis([0 inf -0.4 1])
subplot(2,2,4)
hold on
plot(dt, tool_data(:,4), 'LineWidth', 2, 'DisplayName','Xr');
plot(dt, tool_data(:,5), 'LineWidth', 2, 'DisplayName','Yr');
plot(dt, tool_data(:,6), 'LineWidth', 2, 'DisplayName','Zr');
hold off
title('Tool Rotation')
ylabel('Rotation [radians]')
xlabel('Time [s]')
legend('show')
axis([0 inf -3.5 3.5])