% MyRotobrush.m  - UMD CMSC426, Fall 2018
% This is the main script of your rotobrush project.
% We've included an outline of what you should be doing, and some helful visualizations.
% However many of the most important functions are left for you to implement.
% Feel free to modify this code as you see fit.
clear all;
% Some parameters you need to tune:
WindowWidth = 40;  
ProbMaskThreshold = -1; 
NumWindows= 20; 
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
mask1 = load('mask1.mat');
mask = mask1.mask;
imshow(imoverlay(images{1}, boundarymask(mask,8),'red'));
set(gca,'position',[0 0 1 1],'units','normalized')
% F = getframe(gcf);
% [I,~] = frame2im(F);
% imwrite(I, fullfile(fpath, strip(imageNames(1,:))));
% outputVideo = VideoWriter(fullfile(fpath,'video.mp4'),'MPEG-4');
% open(outputVideo);
% writeVideo(outputVideo,I);

% Sample local windows and initialize shape+color models:
[mask_outline, LocalWindows] = initLocalWindows(images{1},mask,NumWindows,WindowWidth,true);

s = size(LocalWindows,1);
%ColorModels = {};
% Must define a field ColorModels.Confidences: a cell array of the color confidence map for each local window.


    %K = rgb2lab(IMG);
    %Mask = rgb2lab(Mask);
    %I1 = K*Mask;
    local_windows = get_local_windows(images{1}, LocalWindows, WindowWidth/2);
    %I2 = K*(~Mask);
    local_mask = get_local_windows(mask, LocalWindows, WindowWidth/2);

    %s = size(LocalWindows,1);
    combined_color_prob_cell = cell(1,s);
    foreground_model_cell = cell(1,s);
    background_model_cell = cell(1,s);
    options = statset('MaxIter',500);
    
    for i = 1:s
        curr_image = local_windows{1,i};
        inverted = local_mask{1,i}==0; %background becomes non zero
        
        [r c] = find(bwdist(inverted) > 5);
        foreground_pix = rgb2lab(impixel(curr_image,c,r));
        foreground_model = fitgmdist(foreground_pix,3,'RegularizationValue',0.001,'Options',options);
        foreground_model_cell{1,i} = foreground_model;
        
        [r c] = find(bwdist(local_mask{1,i})>5); %find all non zero that are more than 2 away from foreground 
        background_pix = rgb2lab(impixel(curr_image,c,r));
        background_model = fitgmdist(background_pix,3,'RegularizationValue',0.001,'Options',options);
        background_model_cell{1,i} = background_model;
        
        % probs
        [r c ~] = size(curr_image);
        values = rgb2lab(reshape(double(curr_image),[r*c 3]));

        fore_prob = pdf(foreground_model,values);
        back_prob = pdf(background_model,values);

        comb_prob = fore_prob./(fore_prob+back_prob);
        combined_color_prob_cell{1,i} = reshape(comb_prob,[r c]);
        ColorModels{i}.Confidence = comb_prob;
        imshow(combined_color_prob_cell{1,i});
    end


% ColorModels = ...
%     initColorModels(images{1},mask,mask_outline,LocalWindows,BoundaryWidth,WindowWidth);

% You should set these parameters yourself:
fcutoff = -1;
SigmaMin = -1;
SigmaMax = -1;
R = -1;
A = -1;
ShapeConfidences = ...
    initShapeConfidences(LocalWindows,ColorModels,...
    WindowWidth, SigmaMin, A, fcutoff, R);

% Show initial local windows and output of the color model:
imshow(images{1})
hold on
showLocalWindows(LocalWindows,WindowWidth,'r.');
hold off
set(gca,'position',[0 0 1 1],'units','normalized')
F = getframe(gcf);
[I,~] = frame2im(F);

showColorConfidences(images{1},mask_outline,ColorModels.Confidences,LocalWindows,WindowWidth);

%%% MAIN LOOP %%%
% Process each frame in the video.
for prev=1:(length(files)-1)
    curr = prev+1;
    fprintf('Current frame: %i\n', curr)
    
    %%% Global affine transform between previous and current frames:
    [warpedFrame, warpedMask, warpedMaskOutline, warpedLocalWindows] = calculateGlobalAffine(images{prev}, images{curr}, mask, LocalWindows);
    
    %%% Calculate and apply local warping based on optical flow:
    NewLocalWindows = ...
        localFlowWarp(warpedFrame, images{curr}, warpedLocalWindows,warpedMask,WindowWidth);
    
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
        LocalWindows, ...
        ColorModels, ...
        ShapeConfidences, ...
    ] = ...
    updateModels(...
        NewLocalWindows, ...
        LocalWindows, ...
        images{curr}, ...
        warpedMask, ...
        warpedMaskOutline, ...
        WindowWidth, ...
        ColorModels, ...
        ShapeConfidences, ...
        ProbMaskThreshold, ...
        fcutoff, ...
        SigmaMin, ...
        R, ...
        A ...
    );

    mask_outline = bwperim(mask,4);

    % Write video frame:
    imshow(imoverlay(images{curr}, boundarymask(mask,8), 'red'));
    set(gca,'position',[0 0 1 1],'units','normalized')
    F = getframe(gcf);
    [I,~] = frame2im(F);
    imwrite(I, fullfile(fpath, strip(imageNames(curr,:))));
    writeVideo(outputVideo,I);

    imshow(images{curr})
    hold on
    showLocalWindows(LocalWindows,WindowWidth,'r.');
    hold off
    set(gca,'position',[0 0 1 1],'units','normalized')
    F = getframe(gcf);
    [I,~] = frame2im(F);
end

close(outputVideo);

