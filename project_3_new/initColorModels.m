function [ColorModels, local_mask] = initColorModels(IMG, Mask, MaskOutline, LocalWindows, WindowWidth)
% INITIALIZAECOLORMODELS Initialize color models.  ColorModels is a struct you should define yourself.

    s = size(LocalWindows,1);

% Must define a field ColorModels.Confidences: a cell array of the color confidence map for each local window.
    local_windows = get_local_windows(IMG, LocalWindows, WindowWidth/2);
    local_mask = get_local_windows(Mask, LocalWindows, WindowWidth/2);

    combined_color_prob_cell = cell(1,s);
    foreground_model_cell = cell(1,s);
    background_model_cell = cell(1,s);
    options = statset('MaxIter',500);
    
    for i = 1:s
        curr_image = local_windows{1,i};
        inverted = local_mask{1,i}==0; %background becomes non zero
        
        [r c] = find(bwdist(inverted) > 5);
        foreground_pix = rgb2lab(impixel(curr_image,c,r));
        foreground_model = fitgmdist(foreground_pix, 3,'RegularizationValue',0.001,'Options',options);
        foreground_model_cell{1,i} = foreground_model;
        ColorModels{i}.foreground_model_cell = foreground_model;
        
        [r c] = find(bwdist(local_mask{1,i})>5); %find all non zero that are more than 2 away from foreground 
        background_pix = rgb2lab(impixel(curr_image,c,r));
        background_model = fitgmdist(background_pix, 3,'RegularizationValue',0.001,'Options',options);
        background_model_cell{1,i} = background_model;
        ColorModels{i}.background_model_cell = background_model;
        
        % probs
        [r c ~] = size(curr_image);
        values = rgb2lab(reshape(double(curr_image),[r*c 3]));

        fore_prob = pdf(foreground_model,values);
        back_prob = pdf(background_model,values);

        comb_prob = fore_prob./(fore_prob+back_prob);
        combined_color_prob_cell{1,i} = reshape(comb_prob,[r c]);
        
        d = bwdist(local_mask{1,i});
        wc = exp(-d.^2./(WindowWidth/2).^2);
        numer_window = abs(local_mask{1,i} - combined_color_prob_cell{1,i}).*wc; 
        numer = sum(numer_window(:));
        denom = sum(wc(:));
        ColorModels{i}.Confidence = 1 - numer/denom;
        ColorModels{i}.combined = combined_color_prob_cell{1,i};
        %Fore_prob = combined_color_prob_cell;
        %imshow(combined_color_prob_cell{1,i});
    end
end

