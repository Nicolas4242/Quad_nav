function ax12a_joint_mode_one_motor_corrected()
%% =========================================================
% AX-12A - Joint Mode - One Motor
% Position ref/measured + speed ref/measured on same plot
%
% Figure 1:
%   Position in DXL units
%   Speed in DXL units
%
% Figure 2:
%   Position in degrees
%   Speed in rpm
%% =========================================================
clc; close all;

%% 1) SETTINGS
DXL_ID = 11;
PROTOCOL_VERSION = 1.0;
BAUDRATE = 1000000;
DEVICENAME = 'COM10';

ADDR_CW_ANGLE_LIMIT   = 6;
ADDR_CCW_ANGLE_LIMIT  = 8;
ADDR_TORQUE_ENABLE    = 24;
ADDR_GOAL_POSITION    = 30;
ADDR_MOVING_SPEED     = 32;
ADDR_PRESENT_POSITION = 36;
ADDR_PRESENT_SPEED    = 38;

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;

CW_LIMIT  = 0;
CCW_LIMIT = 1023;

% AX-12A Joint Mode position conversion
AX_MAX_RAW_POSITION = 1023;
AX_MAX_DEGREE = 300.0;

% AX-12A speed conversion
% 1 DXL speed unit = 0.111 rpm
AX_SPEED_UNIT_TO_RPM = 0.111;

%% 2) LOAD LIBRARY
if strcmp(computer, 'PCWIN')
    LIB_NAME = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    LIB_NAME = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    LIB_NAME = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    LIB_NAME = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    LIB_NAME = 'libdxl_mac_c';
else
    error('Unsupported OS');
end

if ~libisloaded(LIB_NAME)
    loadlibrary(LIB_NAME, 'dynamixel_sdk.h', ...
        'addheader', 'port_handler.h', ...
        'addheader', 'packet_handler.h');
end

port_num = portHandler(DEVICENAME);
packetHandler();

cleanupObj = onCleanup(@() localCleanup( ...
    port_num, PROTOCOL_VERSION, DXL_ID, ...
    ADDR_TORQUE_ENABLE, TORQUE_DISABLE, LIB_NAME));

closePort(port_num);

if ~openPort(port_num)
    error('Cannot open port.');
end

if ~setBaudRate(port_num, BAUDRATE)
    error('Cannot set baudrate.');
end

fprintf('Port opened successfully.\n');

%% 3) CONFIGURE JOINT MODE
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_CW_ANGLE_LIMIT, CW_LIMIT);
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_CCW_ANGLE_LIMIT, CCW_LIMIT);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

fprintf('Joint mode enabled. Torque ON.\n');

%% 4) TEST PARAMETERS
start_position = 100;
target_position = 700;

speed_limits = [100 200 300 500];

return_speed = 150;      % speed used to return to position 100
move_duration = 3.0;     % acquisition duration toward 700
return_duration = 2.0;   % acquisition duration returning to 100
Ts = 0.02;

%% 5) DATA STORAGE
time_data       = [];
pos_ref_data    = [];
pos_meas_data   = [];
speed_ref_data  = [];
speed_meas_data = [];

global_t0 = tic;

%% 6) GO TO INITIAL POSITION
fprintf('Moving to initial position...\n');
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOVING_SPEED, return_speed);
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, start_position);
pause(2.5);

%% 7) MAIN LOOP
for i = 1:length(speed_limits)
    speed_limit = speed_limits(i);

    fprintf('\n=== Test %d / %d | Speed limit = %d ===\n', ...
        i, length(speed_limits), speed_limit);

    % -------------------------------------------------
    % A) Return to position 100
    % -------------------------------------------------
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOVING_SPEED, return_speed);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, start_position);

    t_return = tic;
    while toc(t_return) < return_duration
        t_now = toc(global_t0);

        raw_pos   = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        raw_speed = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_SPEED);

        meas_speed_signed = decodePresentSpeed(raw_speed);

        time_data(end+1,1)       = t_now;
        pos_ref_data(end+1,1)    = start_position;
        pos_meas_data(end+1,1)   = raw_pos;
        speed_ref_data(end+1,1)  = return_speed;
        speed_meas_data(end+1,1) = meas_speed_signed;

        pause(Ts);
    end

    % -------------------------------------------------
    % B) Move toward position 700
    % -------------------------------------------------
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOVING_SPEED, speed_limit);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, target_position);

    t_move = tic;
    while toc(t_move) < move_duration
        t_now = toc(global_t0);

        raw_pos   = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        raw_speed = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_SPEED);

        meas_speed_signed = decodePresentSpeed(raw_speed);

        time_data(end+1,1)       = t_now;
        pos_ref_data(end+1,1)    = target_position;
        pos_meas_data(end+1,1)   = raw_pos;
        speed_ref_data(end+1,1)  = speed_limit;
        speed_meas_data(end+1,1) = meas_speed_signed;

        pause(Ts);
    end
end

%% 8) CONVERSIONS
% Position conversion:
% AX-12A Joint Mode:
% 0 raw units    = 0 degrees
% 1023 raw units = 300 degrees
pos_ref_deg  = pos_ref_data  * AX_MAX_DEGREE / AX_MAX_RAW_POSITION;
pos_meas_deg = pos_meas_data * AX_MAX_DEGREE / AX_MAX_RAW_POSITION;

% Speed conversion:
% 1 DXL speed unit = 0.111 rpm
speed_ref_rpm  = speed_ref_data  * AX_SPEED_UNIT_TO_RPM;
speed_meas_rpm = speed_meas_data * AX_SPEED_UNIT_TO_RPM;

%% 9) COMBINED PLOT IN DXL UNITS
figure;
grid on; hold on;

yyaxis left
stairs(time_data, pos_ref_data, 'LineWidth', 2);
plot(time_data, pos_meas_data, '--', 'LineWidth', 1.5);
ylabel('Position [DXL units]');

yyaxis right
stairs(time_data, speed_ref_data, 'LineWidth', 2);
plot(time_data, speed_meas_data, '--', 'LineWidth', 1.5);
ylabel('Speed [DXL units]');
yline(0, ':');

xlabel('Time [s]');
title('Position and speed vs time - DXL units');
legend('Position Ref', 'Position Meas', 'Speed Ref', 'Speed Meas', ...
    'Location', 'best');

%% 10) COMBINED PLOT IN DEGREES AND RPM
figure;
grid on; hold on;

yyaxis left
stairs(time_data, pos_ref_deg, 'LineWidth', 2);
plot(time_data, pos_meas_deg, '--', 'LineWidth', 1.5);
ylabel('Position [degrees]');

yyaxis right
stairs(time_data, speed_ref_rpm, 'LineWidth', 2);
plot(time_data, speed_meas_rpm, '--', 'LineWidth', 1.5);
ylabel('Speed [rpm]');
yline(0, ':');

xlabel('Time [s]');
title('Position and speed vs time - Position in degrees, speed in rpm');
legend('Position Ref [deg]', 'Position Meas [deg]', ...
       'Speed Ref [rpm]', 'Speed Meas [rpm]', ...
       'Location', 'best');

fprintf('\nTest finished.\n');

end

%% =========================================================
% Helper functions
%% =========================================================

function speed_signed = decodePresentSpeed(raw_speed)
    if raw_speed < 1024
        speed_signed = raw_speed;
    else
        speed_signed = -(raw_speed - 1024);
    end
end

function localCleanup(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, LIB_NAME)
    fprintf('\nCleaning up...\n');
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    closePort(port_num);

    if libisloaded(LIB_NAME)
        unloadlibrary(LIB_NAME);
    end
end