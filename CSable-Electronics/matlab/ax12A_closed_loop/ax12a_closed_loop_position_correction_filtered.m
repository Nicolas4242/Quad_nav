function ax12a_closed_loop_position_correction_filtered()
%% =========================================================
% AX-12A - Joint Mode - Closed-loop position correction
% With filtered measured velocity and filtered command velocity
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
CCW_LIMIT = 1023;   % joint mode

Ts = 0.01;          % sampling period
T_total = 10;       % total test time [s]

Kp = 6;             % position correction gain

% -------- Filters --------
alpha_cmd  = 0.85;  % command smoothing (0.8 to 0.95)
beta_meas  = 0.80;  % measured velocity smoothing (0.7 to 0.9)

% trajectory limits
q_min = 200;
q_max = 700;
A = (q_max - q_min)/2;      % amplitude
q0 = (q_max + q_min)/2;     % center

% sinusoidal reference
f = 0.16;                   % Hz
w = 2*pi*f;

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

%% 3) JOINT MODE + TORQUE
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_CW_ANGLE_LIMIT, CW_LIMIT);
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_CCW_ANGLE_LIMIT, CCW_LIMIT);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

fprintf('Joint mode enabled. Torque ON.\n');

%% 4) MOVE TO START POSITION
q_start = round(q0);
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOVING_SPEED, 100);
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, q_start);
pause(2.0);

%% 5) DATA STORAGE
time_data        = [];
q_ref_data       = [];
q_meas_data      = [];
v_ref_data       = [];
v_cmd_raw_data   = [];
v_cmd_filt_data  = [];
v_meas_data      = [];
v_meas_filt_data = [];
err_data         = [];

%% 6) FILTER STATES
v_cmd_filt_prev  = 100;   % initial filtered command
v_meas_filt_prev = 0;     % initial filtered measured speed

%% 7) MAIN LOOP
t0 = tic;

while true
    t = toc(t0);
    if t > T_total
        break;
    end

    % ----- reference trajectory -----
    % q_ref in DXL units
    q_ref = q0 + A * sin(w*t);

    % derivative of q_ref in DXL units/s
    v_ref = A * w * cos(w*t);

    % ----- measurements -----
    raw_q = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_POSITION);
    raw_v = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRESENT_SPEED);

    q_meas = raw_q;
    v_meas_signed = decodePresentSpeed(raw_v);

    % ----- filter measured velocity -----
    v_meas_filt = beta_meas * v_meas_filt_prev + (1 - beta_meas) * v_meas_signed;
    v_meas_filt_prev = v_meas_filt;

    % ----- position error -----
    e_q = q_ref - q_meas;

    % ----- reference velocity command -----
    % Keep your original idea, but smoother behavior comes mainly from filtering
    v_ref_cmd = abs(v_ref);

    % ----- raw command -----
    v_cmd_raw = v_ref_cmd + Kp * abs(e_q);

    % ----- saturation before filter -----
    v_cmd_raw = max(1, min(1023, v_cmd_raw));

    % ----- filter commanded velocity -----
    v_cmd_filt = alpha_cmd * v_cmd_filt_prev + (1 - alpha_cmd) * v_cmd_raw;
    v_cmd_filt_prev = v_cmd_filt;

    % ----- final integer command -----
    v_cmd_send = round(max(1, min(1023, v_cmd_filt)));

    % ----- send commands -----
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_GOAL_POSITION, round(q_ref));
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MOVING_SPEED, v_cmd_send);

    % ----- store -----
    time_data(end+1,1)        = t;
    q_ref_data(end+1,1)       = q_ref;
    q_meas_data(end+1,1)      = q_meas;
    v_ref_data(end+1,1)       = v_ref_cmd;
    v_cmd_raw_data(end+1,1)   = v_cmd_raw;
    v_cmd_filt_data(end+1,1)  = v_cmd_send;
    v_meas_data(end+1,1)      = v_meas_signed;
    v_meas_filt_data(end+1,1) = v_meas_filt;
    err_data(end+1,1)         = e_q;

    pause(Ts);
end

%% 8) CONVERT SPEED TO RPM
v_meas_rpm      = v_meas_data * 0.111;
v_meas_filt_rpm = v_meas_filt_data * 0.111;
v_cmd_raw_rpm   = v_cmd_raw_data * 0.111;
v_cmd_filt_rpm  = v_cmd_filt_data * 0.111;
v_ref_rpm       = v_ref_data * 0.111;

%% 9) PLOTS

% ---------------------------------------------------------
% Figure 1: Position + filtered command + filtered measured speed
% ---------------------------------------------------------
figure;
grid on; hold on;

yyaxis left
plot(time_data, q_ref_data, 'LineWidth', 2);
plot(time_data, q_meas_data, '--', 'LineWidth', 1.5);
ylabel('Position [DXL units]');

yyaxis right
plot(time_data, v_cmd_filt_data, 'LineWidth', 2);
plot(time_data, v_meas_filt_data, '--', 'LineWidth', 1.5);
ylabel('Speed [DXL units]');

xlabel('Time [s]');
title('Closed-loop correction: position and speed (filtered)');
legend('q_{ref}', 'q_{meas}', 'v_{cmd,filt}', 'v_{meas,filt}', 'Location', 'best');

% ---------------------------------------------------------
% Figure 2: Position error
% ---------------------------------------------------------
err_deg = err_data * (300/1023);

figure('Name','Position Error');

subplot(2,1,1);
plot(time_data, err_data, 'LineWidth', 2);
grid on;
ylabel('Error [DXL]');
title('Position Error');
yline(0,'--');

subplot(2,1,2);
plot(time_data, err_deg, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Error [deg]');
title('Position Error (Degrees)');
yline(0,'--');

% ---------------------------------------------------------
% Figure 3: Speed comparison in rpm
% ---------------------------------------------------------
figure;
grid on; hold on;
plot(time_data, v_ref_rpm, 'LineWidth', 1.5);
plot(time_data, v_cmd_raw_rpm, 'LineWidth', 1.2);
plot(time_data, v_cmd_filt_rpm, 'LineWidth', 2);
plot(time_data, v_meas_rpm, '--', 'LineWidth', 1.2);
plot(time_data, v_meas_filt_rpm, '--', 'LineWidth', 2);

xlabel('Time [s]');
ylabel('Speed [rpm]');
title('Reference speed, raw/filtered command, raw/filtered measured speed');
legend('v_{ref}', 'v_{cmd,raw}', 'v_{cmd,filt}', 'v_{meas,raw}', 'v_{meas,filt}', ...
    'Location', 'best');

fprintf('\nFinished.\n');
fprintf('Kp = %.2f\n', Kp);
fprintf('alpha_cmd = %.2f | beta_meas = %.2f\n', alpha_cmd, beta_meas);
fprintf('If motor still vibrates, increase alpha_cmd or reduce Kp.\n');
fprintf('Example: alpha_cmd = 0.90 or Kp = 4.\n');

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