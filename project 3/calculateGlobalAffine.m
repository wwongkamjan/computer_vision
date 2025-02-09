function [WarpedFrame, WarpedMask, WarpedMaskOutline, WarpedLocalWindows] = calculateGlobalAffine(IMG1,IMG2,Mask,Windows, WindowWidth)
% CALCULATEGLOBALAFFINE: finds affine transform between two frames, and applies it to frame1, the mask, and local windows.
 R = imref2d(size(Mask));
    halfw = WindowWidth/2;
    s = size(Windows, 1);
        %calculate transformation
        gray_im1 = rgb2gray(IMG1);
        gray_im2 = rgb2gray(IMG2);
        
        % crop the images to only around the target area
%         Windows = reshape(cell2mat(Windows(:)),[s 2]);
         minc = uint32(min(Windows(:,1))); maxc = uint32(max(Windows(:,1))); 
         minr = uint32(min(Windows(:,2))); maxr = uint32(max(Windows(:,2))); 
         gray_im1 =gray_im1(minr-halfw-5:maxr+halfw+5,minc-halfw-5:maxc+halfw+5);
         gray_im2 =gray_im2(minr-halfw-5:maxr+halfw+5,minc-halfw-5:maxc+halfw+5);

        %match stuff
        points1 = detectSURFFeatures(gray_im1,'MetricThreshold',100);
        points2 = detectSURFFeatures(gray_im2,'MetricThreshold',100);
        [features1, validpts1]  = extractFeatures(gray_im1,points1);
        [features2, validpts2] = extractFeatures(gray_im2,points2);
        indexPairs = matchFeatures(features1,features2);
        matchedPoints1 = validpts1(indexPairs(:,1));
        matchedPoints2 = validpts2(indexPairs(:,2));
        showMatchedFeatures(gray_im1,gray_im2, matchedPoints1, matchedPoints2,'montage');
        my_transformation= estimateGeometricTransform(matchedPoints2,matchedPoints1,'affine');
        WarpedFrame = imwarp(IMG1, my_transformation,'OutputView',R);
        WarpedMaskOutline = bwperim(Mask, 8);
        %WarpedLocalWindows = imwarp(Mask, invert(my_transformation), 'OutputView', R);
        WarpedMask = imwarp(Mask, my_transformation, 'OutputView', R);
        WarpedLocalWindows = zeros(s, 2);
        for i = 1:s
            mycenter = Windows(i,:);
            cr = double(mycenter(1)); 
            cc = double(mycenter(2));
            [newr,newc] = transformPointsForward(invert(my_transformation), cr, cc);
            WarpedLocalWindows(i,:) = double([newr newc]);
        end
end

