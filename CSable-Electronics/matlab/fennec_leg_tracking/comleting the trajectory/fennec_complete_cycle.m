%% Fennec foot trajectory smoothing + complete cycle + FINAL EDITING
% This script:
% 1. Reads tracked foot-tip points
% 2. Smooths the swing trajectory
% 3. Completes the bottom ground-contact phase
% 4. Shows the FINAL result
% 5. Lets you edit the FINAL smooth curve
% 6. Saves the edited final trajectory

clear; clc; close all;

%% ================= USER SETTINGS =================

if isfile('fennec_foot_points.csv')
    inputCsv = 'fennec_foot_points.csv';
elseif isfile('fennec_foot_points_preview.csv')
    inputCsv = 'fennec_foot_points_preview.csv';
else
    inputCsv = '';
end

nSwing  = 100;
nStance = 80;

smoothWindow = 9;

scale_mm_per_px = 1;

% Edit the final final result after smoothing
doFinalEdit = true;

% How many neighboring points move with the selected point
% Bigger = smoother local edit
editRadiusPoints = 8;

%% ================= LOAD POINTS =================

if ~isempty(inputCsv)
    T = readtable(inputCsv);
    x_px = T.x_px;
    y_px = T.y_px;
else
    x_px = [512; 482; 461; 448; 432; 426; 423; 422; 422; 422; 423];
    y_px = [340; 324; 321; 322; 330; 333; 335; 336; 337; 337; 337];
end

valid = ~isnan(x_px) & ~isnan(y_px);
x_px = x_px(valid);
y_px = y_px(valid);

%% ================= CONVERT TO ROBOT COORDINATES =================

ground_y = max(y_px);
z_px = ground_y - y_px;

x_rel = x_px - min(x_px);

if x_rel(1) > x_rel(end)
    x_rel = flipud(x_rel);
    z_px  = flipud(z_px);
end

z_px(1) = 0;
z_px(end) = 0;

x_rel = x_rel - x_rel(1);
stride_px = x_rel(end);

%% ================= SMOOTH SWING PHASE =================

s_raw = linspace(0, 1, length(x_rel))';
s_fine = linspace(0, 1, nSwing)';

x_swing = interp1(s_raw, x_rel, s_fine, 'pchip');
z_swing = interp1(s_raw, z_px,  s_fine, 'pchip');

x_swing = movingAverageCustom(x_swing, smoothWindow);
z_swing = movingAverageCustom(z_swing, smoothWindow);

z_swing = max(z_swing, 0);

x_swing(1) = 0;
x_swing(end) = stride_px;
z_swing(1) = 0;
z_swing(end) = 0;

%% ================= COMPLETE THE CYCLE =================

x_stance = linspace(stride_px, 0, nStance)';
z_stance = zeros(nStance, 1);

x_cycle_px = [x_swing; x_stance(2:end)];
z_cycle_px = [z_swing; z_stance(2:end)];

%% ================= EDIT FINAL FINAL RESULT =================

if doFinalEdit
    [x_cycle_px, z_cycle_px] = editFinalSmoothTrajectory( ...
        x_cycle_px, z_cycle_px, editRadiusPoints);
end

% Keep foot above ground
z_cycle_px = max(z_cycle_px, 0);

% Convert to mm after final editing
x_cycle_mm = x_cycle_px * scale_mm_per_px;
z_cycle_mm = z_cycle_px * scale_mm_per_px;

phase = [repmat({'swing'}, nSwing, 1); repmat({'stance'}, length(x_cycle_px)-nSwing, 1)];

%% ================= SAVE OUTPUT CSV =================

cyclePoint = (1:length(x_cycle_px))';

Tout = table(cyclePoint, phase, x_cycle_px, z_cycle_px, x_cycle_mm, z_cycle_mm, ...
    'VariableNames', {'point','phase','x_px','z_px','x_mm','z_mm'});

writetable(Tout, 'fennec_FINAL_EDITED_foot_cycle.csv');

%% ================= PLOT FINAL EDITED RESULT =================

figure('Color','w');
plot(x_cycle_px, z_cycle_px, 'b-', 'LineWidth', 3); hold on;
plot(x_cycle_px, z_cycle_px, 'bo', 'MarkerSize', 3);

grid on;
axis equal;
xlabel('x position [pixels]');
ylabel('foot height z [pixels]');
title('FINAL edited smooth robot foot cycle');

saveas(gcf, 'fennec_FINAL_EDITED_foot_cycle.png');

fprintf('Done. Files saved:\n');
fprintf('  fennec_FINAL_EDITED_foot_cycle.csv\n');
fprintf('  fennec_FINAL_EDITED_foot_cycle.png\n');

%% ================= LOCAL FUNCTIONS =================

function [xEdited, zEdited] = editFinalSmoothTrajectory(x, z, radiusPts)
    % editFinalSmoothTrajectory edits the final smooth result.
    %
    % How to use:
    % 1. Click near the final curve point you want to edit.
    % 2. Click the new position.
    % 3. Nearby points move smoothly with it.
    % 4. Press ENTER to finish.

    xEdited = x;
    zEdited = z;

    keepEditing = true;

    while keepEditing
        figure(200); clf;

        plot(xEdited, zEdited, 'b-', 'LineWidth', 3); hold on;
        plot(xEdited, zEdited, 'bo', 'MarkerSize', 3);

        grid on;
        axis equal;
        xlabel('x position [pixels]');
        ylabel('foot height z [pixels]');
        title({'FINAL trajectory editing', ...
               'Click a point on the final curve, then click its new position.', ...
               'Nearby points will move smoothly. Press ENTER to finish.'});

        fprintf('\nClick near the FINAL curve point you want to edit.\n');
        fprintf('Press ENTER to finish editing.\n');

        [xClick, zClick, button] = ginput(1);

        if isempty(xClick) || isempty(button)
            keepEditing = false;
            break;
        end

        distances = sqrt((xEdited - xClick).^2 + (zEdited - zClick).^2);
        [~, idx] = min(distances);

        fprintf('Selected final point %d.\n', idx);
        fprintf('Now click the new position.\n');

        plot(xEdited(idx), zEdited(idx), 'ro', 'MarkerSize', 14, 'LineWidth', 2);

        [xNew, zNew, button2] = ginput(1);

        if isempty(xNew) || isempty(button2)
            keepEditing = false;
            break;
        end

        dx = xNew - xEdited(idx);
        dz = zNew - zEdited(idx);

        % Smooth local influence around selected point
        n = length(xEdited);
        indices = (1:n)';

        distanceFromSelected = abs(indices - idx);

        weights = zeros(n, 1);
        affected = distanceFromSelected <= radiusPts;

        % Cosine weights:
        % selected point moves 100%
        % neighbors move gradually less
        weights(affected) = 0.5 * (1 + cos(pi * distanceFromSelected(affected) / radiusPts));

        xEdited = xEdited + dx * weights;
        zEdited = zEdited + dz * weights;

        % Prevent foot from going below ground
        zEdited = max(zEdited, 0);

        fprintf('Edited point %d with smooth local correction.\n', idx);
    end

    close(200);
end

function ySmooth = movingAverageCustom(y, windowSize)
    % movingAverageCustom smooths a vector using a simple moving average.

    if windowSize <= 1
        ySmooth = y;
        return;
    end

    windowSize = round(windowSize);

    if mod(windowSize, 2) == 0
        windowSize = windowSize + 1;
    end

    halfWindow = floor(windowSize / 2);

    yPadded = [repmat(y(1), halfWindow, 1); y; repmat(y(end), halfWindow, 1)];
    kernel = ones(windowSize, 1) / windowSize;

    ySmooth = conv(yPadded, kernel, 'valid');
end