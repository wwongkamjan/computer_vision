function [mask, LocalWindows, ColorModels, ShapeConfidences] = ...
    updateModels(...
        NewLocalWindows, ...
        LocalWindows, ...
        CurrentFrame, ...
        warpedMask, ...
        warpedMaskOutline, ...
        WindowWidth, ...
        ColorModels, ...
        ShapeConfidences, ...
        ~, ...
        fcutoff, ...
        SigmaMin, ...
        R, ...
        A ...
    )
% UPDATEMODELS: update shape and color models, and apply the result to generate a new mask.
% Feel free to redefine this as several different functions if you prefer.
%hint: update based on area of new foreground mask
    s = size(LocalWindows,1);
    combined_color_prob_cell2 = cell(1,s);
    foreground_model_cell = cell(1,s);
    background_model_cell = cell(1,s);
    options = statset('MaxIter',500);
    local_mask = get_local_windows(warpedMask, LocalWindows, WindowWidth/2);
    local_windows = get_local_windows(CurrentFrame, LocalWindows, WindowWidth/2);
    for i = 1:s
        inverted = local_mask{1,i}==0;
        [r c] = find(bwdist(inverted) > 5);
        foreground_pix = rgb2lab(impixel(local_windows{1,i},c,r));
        [a b] = size(foreground_pix);
        if a < 100 %if not enough data so loosen boundary
            [r c] = find(bwdist(inverted) > 5);
            foreground_pix = rgb2lab(impixel(local_windows{1,i},c,r));
            [a b] = size(foreground_pix);
        end
        if a > b
            foreground_model = fitgmdist(foreground_pix,3,'RegularizationValue',0.001,'Options',options);
        else
            foreground_model =  ColorModels{i}.foreground_model_cell;
        end

        [r c] = find(bwdist(local_mask{1,i}) > 5);
        background_pix = rgb2lab(impixel(local_windows{1,i},c,r));
        [a b] = size(background_pix);
        if a < 100
            [r c] = find(bwdist(local_mask{1,i})> 5);
            background_pix = rgb2lab(impixel(local_windows{1,i},c,r));
            [a b] = size(background_pix);
        end
        if a > b
            background_model = fitgmdist(background_pix,3,'RegularizationValue',0.001,'Options',options);
        else
            background_model =  ColorModels{i}.background_model_cell;
        end
        
        % probs
        [r c ~] = size(local_windows{1,i});
        values = rgb2lab(reshape(double(local_windows{1,i}),[r*c 3]));

        fore_prob = pdf(foreground_model,values);
        back_prob = pdf(background_model,values);

        comb_prob = fore_prob./(fore_prob+back_prob);
        
        % previous model probs
        fore_prob_prev = pdf(ColorModels{i}.foreground_model_cell,values);
        back_prob_prev = pdf(ColorModels{i}.background_model_cell,values);
        comb_prob_prev = fore_prob_prev./(fore_prob_prev+back_prob_prev);
        %size(comb_prob_prev)
        if sum(comb_prob >0.5) > sum(comb_prob_prev>0.2) % use prev model
            %disp("prev color");
            combined_color_prob_cell2{1,i} = reshape(comb_prob_prev,[r c]); 
            foreground_model_cell{1,i} = ColorModels{i}.foreground_model_cell;
            background_model_cell{1,i} = ColorModels{i}.background_model_cell;
        else
           % disp("new color");
            combined_color_prob_cell2{1,i} = reshape(comb_prob,[r c]); 
            foreground_model_cell{1,i} = foreground_model;
            background_model_cell{1,i} = background_model;
        end
%         imshow(combined_color_prob_cell2{1,i});
%         hold on
%         plot(uint32(c/2), uint32(r/2), 'r*', 'LineWidth', 2, 'MarkerSize', 5);
%         hold off
    end
%-------------------------------------------------------------------------------
   [ColorModels, local_mask] = initializeColorModels(CurrentFrame, warpedMask, warpedMaskOutline, LocalWindows, WindowWidth);
   ShapeConfidences = initShapeConfidences(LocalWindows, ColorModels, warpedMask, WindowWidth, SigmaMin, A, fcutoff, R); 
   total_confidence_cell = get_total_confidence_cell(ShapeConfidences,ColorModels,local_mask,s) ;
    mask = get_final_mask(rgb2gray(CurrentFrame),NewLocalWindows,total_confidence_cell,WindowWidth/2,s);
    mask = mask > 0.5;
    mask = imfill(mask,'holes');
end

