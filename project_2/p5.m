% read an input 

train1_1 = imread('1.jpg');
train1_2 = imread('2.jpg');
train1_3 = imread('3.jpg');

train1_1 = rgb2gray(train1_1);
train1_2 = rgb2gray(train1_2);
train1_3 = rgb2gray(train1_3);


cm1_1 = cornermetric(train1_1);
cm1_2 = cornermetric(train1_2);
cm1_3 = cornermetric(train1_3);


Nbest = 200;
[feature_vector1,x_best_1,y_best_1] = get_feature_descriptor(cm1_1,train1_1);
[feature_vector2,x_best_2,y_best_2] = get_feature_descriptor(cm1_2,train1_2);
k =1;
for i = 1:size(feature_vector1,2)
      first_pt = zeros(64,1);
      first = 1000000000000;
      second = 1000000000000;
    for j = 1:size(feature_vector2,2)
      dist = abs(norm(feature_vector1(:,i) - feature_vector2(:,j)));
        if dist <first
            second = first
            first = dist
            second_pt_index = j;
        elseif dist<second && dist~= first
            second = dist
        end
    end
    if first/second >0.75
        matchedpts1(k,:) = [x_best_1(i),y_best_1(i)];
        matchedpts2(k,:) = [x_best_2(i),y_best_2(i)];
        k = k+1;
    end
    
end

figure; ax = axes;
showMatchedFeatures(train1_1,train1_2,matchedpts1,matchedpts2,'montage','Parent',ax);
title(ax, 'Candidate point matches');
legend(ax, 'Matched points 1','Matched points 2');


    

function [feature_vector,x_best,y_best] = get_feature_descriptor(corner_metric, train_image)
Nbest = 200;
lm1_1 = imregionalmax(corner_metric);


[x,y] = find(lm1_1);

N_Strong = size(x,1);

r = inf(1,N_Strong);
ED = 0;

for i = 1:N_Strong
    for j = 1:N_Strong
        if corner_metric(x(j),y(j)) > corner_metric(x(i),y(i))
           ED =  (x(j)-x(i))^2 + (y(j)-y(i))^2;
           if ED < r(i)
              r(i) = ED;
           end 
        end
    end
end    

[sort_r,index] = sort(r,'descend');


c_best = zeros(2,Nbest);

for i = 1:Nbest
    c_best(1,i) = x(index(i)); 
    c_best(2,i) = y(index(i));
end

x_best = c_best(1,:);
y_best = c_best(2,:);

figure;
imshow(train_image);
hold on;
scatter(y_best,x_best,'r.')
hold off;
H = fspecial('gaussian',40,2);
feature_vector = zeros(64,size(x_best,2));
for i = 1:size(x_best,2)
    x_left = max(x_best(i) -20,1);
    x_right = min(x_best(i)+20,size(train_image,1));
    y_top = max(y_best(i) -20,1);
    y_bottom = min(y_best(i)+20,size(train_image,2));
    new_matrix = train_image(x_left:x_right , y_top:y_bottom);
    convol_matrix = imfilter(new_matrix,H);
    resized_matrix = imresize(convol_matrix,[8,8]);
    resized_matrix = resized_matrix(resized_matrix > -100);
    stand_matrix = (resized_matrix - mean(resized_matrix))/std(double(resized_matrix));
    feature_vector(:,i) = stand_matrix;
end
end
    