function dist = measureDepth(probM)
load training_data.mat

area_1(1) = sum(sum(t1));
area_1(2) = sum(sum(t2));
area_1(3) = sum(sum(t3));
area_1(4) = sum(sum(t5));
area_1(5) = sum(sum(t6));
area_1(6) = sum(sum(t7));
area_1(7) = sum(sum(t8));
area_1(8) = sum(sum(t10));
area_1(9) = sum(sum(t11));
area_1(10) = sum(sum(t12));
area_1(11) = sum(sum(t13));
area_1(12) = sum(sum(t14));
area_1(13) = sum(sum(t16));
area_1(14) = sum(sum(t17));
area_1(15) = sum(sum(t18));
area_1(16) = sum(sum(t19));
area_1(17) = sum(sum(t20));
area_1(18) = sum(sum(t21));
area_1(19) = sum(sum(t22));




dist = [68;76;91;106;114;121;137;152;160;168;176;192;208;216;223;231;248;256;264];
fit_data = fit(area_1',dist,'poly4');
plot(fit_data,area_1',dist);

max1 = max(max(probM));
temp = (probM > max1/100);
temp2 = logical(temp);
stats = regionprops(temp2,'Area');
C = cell2mat(struct2cell(stats));
ball_area = C(1,1);

dist = fit_data(ball_area);
end