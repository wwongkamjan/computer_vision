function ShapeConfidences = initShapeConfidences(LocalWindows, ColorConfidences, Mask, WindowWidth, SigmaMin, A, fcutoff, R)
% INITSHAPECONFIDENCES Initialize shape confidences.  ShapeConfidences is a struct you should define yourself.
s = size(LocalWindows,1);
ShapeConfidences = cell(1,s);
local_mask = get_local_windows(Mask, LocalWindows, WindowWidth/2);

    for i = 1:s
        fc = ColorConfidences{1,i}.Confidence;
        if fc > fcutoff
            sigma_s = SigmaMin + A * (fc - fcutoff)^R;
        else
            sigma_s = SigmaMax;
        end

        d = bwdist(local_mask{1,i});
        ShapeConfidences{1,i} = 1 - exp(-d.^2./sigma_s.^2); %eq 3
        %imshow(shape_model_confidence_mask_cell{1,i});
    end
end
