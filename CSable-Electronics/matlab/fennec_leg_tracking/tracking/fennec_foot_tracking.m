%% Fennec fox foot-tip trajectory from uploaded video
% This script plots the clearest visible front-paw tip trajectory from the video.
% Coordinates are in video pixels: x to the right, y downward, origin at top-left.
% To correct the result, set doManualClick = true and click the foot tip in each frame.

clear; clc; close all;

videoFile = 'WhatsApp Video 2026-05-21 at 3.48.25 PM.mp4';

% Rough tracked points from the clearest visible segment.
% Tracked foot: visible front paw tip during swing/landing.
frames = (8:18)';
x_px = [512; 482; 461; 448; 432; 426; 423; 422; 422; 422; 423];
y_px = [340; 324; 321; 322; 330; 333; 335; 336; 337; 337; 337];

% Set this to true to manually re-click the foot tip frame by frame.
doManualClick = false;

if doManualClick
    v = VideoReader(videoFile);
    [x_px, y_px] = manualTrackFootTip(v, frames);
end

% Save coordinates to CSV.
T = table(frames, x_px, y_px, 'VariableNames', {'frame','x_px','y_px'});
writetable(T, 'fennec_foot_points.csv');

% Plot trajectory as a clean graph.
fig1 = figure('Color','w');
plot(x_px, y_px, '-o', 'LineWidth', 2, 'MarkerSize', 5);
set(gca, 'YDir', 'reverse');
grid on;
axis equal;
xlabel('x position [pixels]');
ylabel('y position [pixels]');
title('Fennec fox visible front-paw tip trajectory');
exportgraphics(fig1, 'fennec_foot_trajectory_plot.png', 'Resolution', 300);

% Overlay trajectory on one video frame.
v = VideoReader(videoFile);
referenceFrame = 14;
frameImg = read(v, referenceFrame + 1); % MATLAB read() is 1-based
fig2 = figure('Color','w');
imshow(frameImg); hold on;
plot(x_px, y_px, '-o', 'LineWidth', 2, 'MarkerSize', 5);
for k = 1:numel(frames)
    text(x_px(k)+4, y_px(k)-4, string(frames(k)), 'Color','y', 'FontSize', 8, 'FontWeight','bold');
end
title('Foot-tip trajectory overlay on video frame');
exportgraphics(fig2, 'fennec_foot_trajectory_overlay.png', 'Resolution', 300);

fprintf('Done. Saved:\n');
fprintf('  fennec_foot_points.csv\n');
fprintf('  fennec_foot_trajectory_plot.png\n');
fprintf('  fennec_foot_trajectory_overlay.png\n');

function [x, y] = manualTrackFootTip(v, frames)
    x = nan(numel(frames),1);
    y = nan(numel(frames),1);

    for k = 1:numel(frames)
        frameNumber = frames(k);
        frameImg = read(v, frameNumber + 1); % MATLAB read() is 1-based

        figure(100); clf;
        imshow(frameImg);
        title(sprintf('Frame %d: click the foot tip, or press Enter to skip', frameNumber));

        [clickedX, clickedY] = ginput(1);
        if ~isempty(clickedX)
            x(k) = clickedX;
            y(k) = clickedY;
        end
    end
end
% manualTrackFootTip shows each selected video frame and stores your clicked foot-tip coordinates.
