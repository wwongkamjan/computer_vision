function mask = snapImage(IMG, foreground)
    L = superpixels(IMG,500);
    f = find(foreground>0.8); b = find(foreground==0);
    mask = lazysnapping(IMG,L,f,b);
end