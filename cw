
function cw = clusterweight(data,p,meank,covk,currentk,maxk)
data = double(data);
sizeM = size(data);
sizeColumn = sizeM(1,2);
cw = zeros(1,sizeColumn);

for i = 1:sizeColumn
    sumscale = 0;
    for j = 1:maxk
    scale = scaledg(data,p, meank, covk,i,j);
    sumscale = sumscale + scale;
    end
    cw(1,i) = scaledg(data,p, meank, covk,i,currentk)/sumscale;
end
