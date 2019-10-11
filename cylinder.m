clear all;
% read an input
% cylindrical -
f = 200;

img = imread('Images/Set1/1.jpg');

cx = size(img,2)/2;

cy = size(img,1)/2;

fx =200;
fy=200;
s=0;
K = [fx s cx; 0 fy cy; 0 0 1];
output=zeros(size(img),'like',img);
if length(size(img))==2
    Layers=1;
else
    Layers=size(img,3);
end
for layer=1:Layers
    x=(1:size(img,2))-cx;
    y=(1:size(img,1))-cy;
    [xx,yy]=meshgrid(x,y);
%     xx=(f.*tan(xx./f))+cx;
%     yy=(yy./(cos(xx./f)))+cy;
     yy=f*yy./sqrt(xx.^2+double(f)^2)+cy;
     xx=f*atan(xx/double(f))+cx;
       
     xx=floor(xx+0.5);
     yy=floor(yy+0.5);

    idx=sub2ind([size(img,1),size(img,2)], yy, xx);

    cylinder_n=zeros(size(img,1),size(img,2),'like',img);
    cylinder_n(idx)=img(:,:,layer);
    output(:,:,layer)=cylinder_n;
end
imshow(output);


