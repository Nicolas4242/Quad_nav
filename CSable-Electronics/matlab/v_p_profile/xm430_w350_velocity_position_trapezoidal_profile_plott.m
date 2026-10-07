function xm430_w350_position_trapezoidal_profile_one_motor()
%% =========================================================
% XM430-W350-R - Position Control Mode - One Motor
% Trapezoidal Profile
%
% Figure 1:
%   Position in DXL units
%   Profile Velocity / Present Velocity in DXL units
%
% Figure 2:
%   Position in degrees
%   Profile Velocity / Present Velocity in rpm
%
% Trapezoidal Profile condition:
%   Profile Velocity     ~= 0
%   Profile Acceleration ~= 0
%% =========================================================
clc; close all;

%% 1) SETTINGS
DXL_ID = 0;                 % Change this if your XM430 has another ID
PROTOCOL_VERSION = 2.0;
BAUDRATE = 1000000;
DEVICENAME = 'COM10';

% EEPROM addresses
ADDR_DRIVE_MODE      = 10;
ADDR_OPERATING_MODE  = 11;

% RAM addresses
ADDR_TORQUE_ENABLE        = 64;
ADDR_PROFILE_ACCELERATION = 108;
ADDR_PROFILE_VELOCITY     = 112;
ADDR_GOAL_POSITION        = 116;
ADDR_PRESENT_VELOCITY     = 128;
ADDR_PRESENT_POSITION     = 132;

TORQUE_ENABLE  = 1;
TORQUE_DISABLE = 0;

% XM430 operating mode
OP_POSITION = 3;

% Drive Mode:
% 0 = Velocity-based Profile + Normal direction
DRIVE_MODE_VELOCITY_BASED = 0;

% Trapezoidal profile:
% Profile Velocity != 0
% Profile Acceleration != 0
PROFILE_ACCELERATION_TRAPEZOIDAL = 20;

% XM430-W350 position conversion
% 1 revolution = 4096 pulses = 360 degrees
XM_MAX_RAW_POSITION = 4096;
XM_MAX_DEGREE = 360.0;
XM_POSITION_UNIT_TO_DEG = XM_MAX_DEGREE / XM_MAX_RAW_POSITION;

% XM430 velocity conversion
% 1 DXL velocity unit = 0.229 rpm
XM_VELOCITY_UNIT_TO_RPM = 0.229;

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

%% 3) CONFIGURE XM430 POSITION MODE + TRAPEZOIDAL PROFILE

% Torque must be OFF before changing EEPROM settings
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);

% Velocity-based profile
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_DRIVE_MODE, DRIVE_MODE_VELOCITY_BASED);

% Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_OPERATING_MODE, OP_POSITION);

% Trapezoidal Profile:
% Profile Acceleration must be non-zero
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION_TRAPEZOIDAL);

% Enable torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

fprintf('XM430 Position Control Mode enabled. Trapezoidal Profile selected. Torque ON.\n');

%% 4) TEST PARAMETERS

% XM430 position range:
% 0 to 4095 DXL units = 0 to 360 degrees
start_position = 500;
target_position = 3000;

% Profile Velocity values
% 1 unit = 0.229 rpm
speed_limits = [30 60 100 150];

return_speed = 80;       % profile velocity used to return to start_position
move_duration = 3.0;     % acquisition duration toward target_position
return_duration = 2.0;   % acquisition duration returning to start_position
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

% Keep trapezoidal profile before movement
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION_TRAPEZOIDAL);

% Set Profile Velocity first, then Goal Position
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, return_speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, start_position);

pause(2.5);

%% 7) MAIN LOOP
for i = 1:length(speed_limits)
    speed_limit = speed_limits(i);

    fprintf('\n=== Test %d / %d | Profile Velocity = %d | Profile Acceleration = %d ===\n', ...
        i, length(speed_limits), speed_limit, PROFILE_ACCELERATION_TRAPEZOIDAL);

    % -------------------------------------------------
    % A) Return to start_position
    % -------------------------------------------------

    % Trapezoidal Profile:
    % Profile Acceleration != 0
    % Profile Velocity != 0
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION_TRAPEZOIDAL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, return_speed);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, start_position);

    t_return = tic;
    while toc(t_return) < return_duration
        t_now = toc(global_t0);

        raw_pos   = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        raw_speed = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_VELOCITY);

        meas_pos_signed   = decodeSigned32(raw_pos);
        meas_speed_signed = decodeSigned32(raw_speed);

        time_data(end+1,1)       = t_now;
        pos_ref_data(end+1,1)    = start_position;
        pos_meas_data(end+1,1)   = meas_pos_signed;
        speed_ref_data(end+1,1)  = return_speed;
        speed_meas_data(end+1,1) = meas_speed_signed;

        pause(Ts);
    end

    % -------------------------------------------------
    % B) Move toward target_position
    % -------------------------------------------------

    % Trapezoidal Profile:
    % Profile Acceleration = non-zero
    % Profile Velocity = speed_limit
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION_TRAPEZOIDAL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PROFILE_VELOCITY, speed_limit);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, target_position);

    t_move = tic;
    while toc(t_move) < move_duration
        t_now = toc(global_t0);

        raw_pos   = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
        raw_speed = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_VELOCITY);

        meas_pos_signed   = decodeSigned32(raw_pos);
        meas_speed_signed = decodeSigned32(raw_speed);

        time_data(end+1,1)       = t_now;
        pos_ref_data(end+1,1)    = target_position;
        pos_meas_data(end+1,1)   = meas_pos_signed;
        speed_ref_data(end+1,1)  = speed_limit;
        speed_meas_data(end+1,1) = meas_speed_signed;

        pause(Ts);
    end
end

%% 8) CONVERSIONS

% Position conversion:
% 0 to 4095 DXL units = 0 to 360 degrees
pos_ref_deg  = pos_ref_data  * XM_POSITION_UNIT_TO_DEG;
pos_meas_deg = pos_meas_data * XM_POSITION_UNIT_TO_DEG;

% Speed conversion:
% 1 DXL velocity unit = 0.229 rpm
speed_ref_rpm  = speed_ref_data  * XM_VELOCITY_UNIT_TO_RPM;
speed_meas_rpm = speed_meas_data * XM_VELOCITY_UNIT_TO_RPM;

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
ylabel('Profile Velocity / Present Velocity [DXL units]');
yline(0, ':');

xlabel('Time [s]');
title('XM430-W350 - Position and velocity vs time - DXL units - Trapezoidal Profile');
legend('Position Ref', 'Position Meas', ...
       'Profile Velocity Ref', 'Present Velocity Meas', ...
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
ylabel('Velocity [rpm]');
yline(0, ':');

xlabel('Time [s]');
title('XM430-W350 - Position in degrees and velocity in rpm - Trapezoidal Profile');
legend('Position Ref [deg]', 'Position Meas [deg]', ...
       'Profile Velocity Ref [rpm]', 'Present Velocity Meas [rpm]', ...
       'Location', 'best');

fprintf('\nTest finished.\n');

end

%% =========================================================
% Helper functions
%% =========================================================

function signed_value = decodeSigned32(raw_value)
    raw_value = double(raw_value);

    if raw_value > 2147483647
        signed_value = raw_value - 4294967296;
    else
        signed_value = raw_value;
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