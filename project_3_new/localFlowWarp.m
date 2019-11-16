function [LocalWindows] = localFlowWarp(WarpedPrevFrame, CurrentFrame, LocalWindows, Mask, Width, local_mask)
% LOCALFLOWWARP Calculate local window movement based on optical flow between frames.

% TODO
%explore farne and kennedy and explain with images the difference between
%the two functions
% CurrentFrame = lab2gray();
% opticFlow = opticalFlowFarneback;
% flow = estimateFlow();
% flow = estimateFlow();

    s = size(LocalWindows, 1);
    halfw = Width/2;
    [myh, myw] = size(rgb2gray(WarpedPrevFrame));
    
    opticFlow = opticalFlowFarneback('NeighborhoodSize',8);
    estimateFlow(opticFlow,rgb2gray(WarpedPrevFrame));
    flow = estimateFlow(opticFlow, rgb2gray(CurrentFrame));
    
    VxWindows = get_local_windows(flow.Vx, LocalWindows, halfw);
    VyWindows = get_local_windows(flow.Vy, LocalWindows, halfw);
    %local_mask = get_local_windows(Mask, LocalWindows, Width/2);
    
    for i = 1:s
        %get mean within foreground
        curr_w_mask = local_mask{1,i};
        currVx = VxWindows{1,i} .*  curr_w_mask;
        Vxmean = sum(currVx(:))/sum(curr_w_mask(:)==1);
        currVy = VyWindows{1,i} .*  curr_w_mask;
        Vymean = sum(currVy(:))/sum(curr_w_mask(:)==1); 
        
        if isnan(Vxmean) || isnan(Vymean)
            Vxmean = 0; Vymean = 0;
        end

        new_center = double(LocalWindows(i,:)) + [Vymean Vxmean];
        if new_center(1)+halfw >= myw || new_center(1)-halfw <= 1 || new_center(2)+halfw >= myh || new_center(2)-halfw <= 1
            if i == 1
                new_center = double(LocalWindows(1,:));
            else
                new_center = double(LocalWindows(randi(i-1),:)); 
            end
        end
        LocalWindows(i, :) = double(new_center);
    end
    %NewLocalWindows = LocalWindows;
end

