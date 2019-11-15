function total_confidence_cell = get_total_confidence_cell(ShapeConfidences,ColorModels,local_mask,s)
    % part 7, combining shape and color models
    total_confidence_cell = cell(1,s);
    for i = 1:s
        fs = ShapeConfidences{i};
        pc = ColorModels{i}.Confidence;
        total_confidence_cell{1,i}= fs.*local_mask{1,i}+(1-fs).*pc;
        %imshow(total_confidence_cell{1,i});
    end
end