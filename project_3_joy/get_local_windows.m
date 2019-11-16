function local_windows = get_local_windows(I,local_windows_center, halfwidth)
    [num_windows, ~] = size(local_windows_center);
    local_windows = cell(1,num_windows);
    for i = 1:num_windows
        center = uint32(local_windows_center(i, :));
        r = center(2); 
        c = center(1);
        %fprintf("r is %d, c is %d\n",r,c);
        my_patch = I(r-halfwidth:r+halfwidth, c-halfwidth:c+halfwidth,:);
        local_windows{1,i} = my_patch; 
        %imshow(my_patch);
    end
end