function foreground2 = get_final_mask(IMG, LocalWindows, total_confidence_cell, halfw, s)
    [h w] = size(IMG); 
    eps = 0.1;
    foreground_numer = zeros([h w]);
    foreground_denom = zeros([h w]);
    %foreground = zeros([h w]);
    for i = 1:s
        center = uint32(LocalWindows(i, :)); cc = center(1); cr = center(2);
        for a = 1:(halfw*2+1)
            for b = 1:(halfw*2+1)
                d = 1.0/(sqrt(double((a-cr).^2+(b-cc).^2))+eps);
                %foreground(cr-halfw+a,cc-halfw+b) =  foreground(cr-halfw+a,cc-halfw+b) + total_confidence_cell{1,i}(a,b)*d;
                foreground_numer(cr-halfw+a,cc-halfw+b) = foreground_numer(cr-halfw+a,cc-halfw+b)+total_confidence_cell{1,i}(a,b)*d;
                foreground_denom(cr-halfw+a,cc-halfw+b) = foreground_denom(cr-halfw+a,cc-halfw+b)+d;
            end
        end
    end
    foreground = foreground_numer./foreground_denom;
    foreground(isnan(foreground))=0;

    L = superpixels(IMG,500);
    f = find(foreground>0); b = find(foreground==0);
    foreground2 = lazysnapping(IMG,L,f,b);
    
end