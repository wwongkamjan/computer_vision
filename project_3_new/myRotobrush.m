% MyRotobrush.m  - UMD CMSC426, Fall 2018
% This is the main script of your rotobrush project.
% We've included an outline of what you should be doing, and some helful visualizations.
% However many of the most important functions are left for you to implement.
% Feel free to modify this code as you see fit.
clear all;
% Some parameters you need to tune:
WindowWidth = 60;  
ProbMaskThreshold = -1; 
NumWindows= 80; 
BoundaryWidth = -1;

% Load images:
fpath = '../input';
files = dir(fullfile(fpath, '*.jpg'));
imageNames = zeros(length(files),1);
images = cell(length(files),1);

for i=1:length(files)
    imageNames(i) = str2double(strtok(files(i).name,'.jpg'));
end

imageNames = sort(imageNames);
imageNames = num2str(imageNames);
imageNames = strcat(imageNames, '.jpg');

for i=1:length(files)
    images{i} = im2double(imread(fullfile(fpath, strip(imageNames(i,:)))));
end

% NOTE: to save time during development, you should save/load your mask rather than use ROIPoly every time.
%mask = roipoly(images{1});
mask = load('mask1.mat');
mask = mask.mask;
imshow(imoverlay(images{1}, boundarymask(mask,8),'red'));
set(gca,'position',[0 0 1 1],'units','normalized')
F = getframe(gcf);
[I,~] = frame2im(F);
imwrite(I, fullfile(fpath, strip(imageNames(1,:))));
outputVideo = VideoWriter(fullfile(fpath,'video.mp4'),'MPEG-4');
open(outputVideo);
writeVideo(outputVideo,I);

% Sample local windows and initialize shape+color models:
[mask_outline, LocalWindows] = initLocalWindows(images{1},mask,NumWindows,WindowWidth,true);
%local_windows = get_local_windows(IMG, LocalWindows, WindowWidth/2);
%local_mask = get_local_windows(mask, LocalWindows, WindowWidth/2);
[ColorModels, local_mask] = ...
    initColorModels(images{1},mask,mask_outline,LocalWindows,WindowWidth);

% You should set these parameters yourself:
fcutoff = 0.6;
SigmaMin = 6;
SigmaMax = WindowWidth+1;
R = 2;
A = (SigmaMax-SigmaMin)/(1-fcutoff)^R;
ShapeConfidences = ...
    initShapeConfidences(LocalWindows,ColorModels, local_mask,...
    SigmaMin, SigmaMax, A, fcutoff, R);

s = size(LocalWindows,1);
total_confidence_cell = get_total_confidence_cell(ShapeConfidences,ColorModels,local_mask, s);

% Show initial local windows and output of the color model:
% imshow(images{1})
% hold on
% showLocalWindows(LocalWindows,WindowWidth,'r.');
% hold off
% set(gca,'position',[0 0 1 1],'units','normalized')
% F = getframe(gcf);
% [I,~] = frame2im(F);
% showColorConfidences(images{1},mask_outline,ColorModels,LocalWindows,WindowWidth);

mask1 = get_final_mask(rgb2gray(images{1}), LocalWindows, total_confidence_cell, WindowWidth/2, s);    
mask = mask1 >0.5;
mask = imfill(mask,'holes');
    
 B = bwboundaries(mask);
imshow(images{1,1});
hold on
for k = 1:length(B)
    boundary = B{k};
    plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
end
hold off
saveas(gcf,sprintf('../output/%d.jpg',1));
close all
figure
    
%%% MAIN LOOP %%%
% Process each frame in the video.
for prev=1:4
%for prev=1:(length(files)-1)
    curr = prev+1;
    fprintf('Current frame: %i\n', curr)
%     hold on;
%     figure
%     imshow(mask)
%     hold off;
    %%% Global affine transform between previous and current frames:
    [warpedFrame, warpedMask, warpedMaskOutline, warpedLocalWindows] = calculateGlobalAffine(images{prev}, images{curr}, mask, LocalWindows, WindowWidth);
    
    %%% Calculate and apply local warping based on optical flow:
    NewLocalWindows = ...
        localFlowWarp(warpedFrame, images{curr}, warpedLocalWindows,warpedMask,WindowWidth, local_mask);
    
    % Show windows before and after optical flow-based warp:
    imshow(images{curr});
    hold on
    showLocalWindows(warpedLocalWindows,WindowWidth,'r.');
    showLocalWindows(NewLocalWindows,WindowWidth,'b.');
    hold off
    
    %%% UPDATE SHAPE AND COLOR MODELS:
    % This is where most things happen.
    % Feel free to redefine this as several different functions if you prefer.
    [ ...
        mask, ...
        mask_outline, ...
        LocalWindows, ...
        ColorModels, ...
        ShapeConfidences, ...
    ] = ...
    updateModels(...
        NewLocalWindows, ...
        LocalWindows, ...
        images{curr}, ...
        mask, ...
        warpedMask, ...
        warpedMaskOutline, ...
        WindowWidth, ...
        ColorModels, ...
        ShapeConfidences, ...
        ProbMaskThreshold, ...
        fcutoff, ...
        SigmaMin, ...
        SigmaMax, ...
        R, ...
        A ...
    );
    
    %mask_outline = bwperim(mask,4);   
   
    B = bwboundaries(mask_outline);
    imshow(images{curr});
    hold on
    for k = 1:length(B)
        boundary = B{k};
        plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
    end
    hold off
    pause(2)
    saveas(gcf,sprintf('../output/%d.jpg',prev));
    
    % Write video frame:
    imshow(imoverlay(images{curr}, boundarymask(mask,8), 'red'));
    set(gca,'position',[0 0 1 1],'units','normalized')
    F = getframe(gcf);
    [I,~] = frame2im(F);
    imwrite(I, fullfile(fpath, strip(imageNames(curr,:))));
   % writeVideo(outputVideo,I);

    imshow(images{curr})
    hold on
    showLocalWindows(LocalWindows,WindowWidth,'r.');
    hold off
    set(gca,'position',[0 0 1 1],'units','normalized')
    F = getframe(gcf);
    [I,~] = frame2im(F);
    %LocalWindows = NewLocalWindows;
end

close(outputVideo);
