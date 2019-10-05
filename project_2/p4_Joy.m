% read an input 

train1_1 = imread('2.jpg');
train1_2 = imread('2.jpg');
train1_3 = imread('3.jpg');

train1_1 = rgb2gray(train1_1);
train1_2 = rgb2gray(train1_2);
train1_3 = rgb2gray(train1_3);


cm1_1 = cornermetric(train1_1);
cm1_2 = cornermetric(train1_2);
cm1_3 = cornermetric(train1_3);


Nbest = 200;

lm1_1 = imregionalmax(cm1_1);


[x,y] = find(lm1_1);

N_Strong = size(x,1);

r = inf(1,N_Strong);
ED = 0;

for i = 1:N_Strong
    for j = 1:N_Strong
        if cm1_1(x(j),y(j)) > cm1_1(x(i),y(i))
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

imshow(train1_1);
hold on;
scatter(y_best,x_best,'r.')
hold off;
H = fspecial('gaussian',40,2);
feature_vector = zeros(64,size(x_best,2));
for i = 1:size(x_best,2)
    x_left = max(x_best(i) -20,1);
    x_right = min(x_best(i)+20,size(train1_1,1));
    y_top = max(y_best(i) -20,1);
    y_bottom = min(y_best(i)+20,size(train1_1,2));
    new_matrix = train1_1(x_left:x_right , y_top:y_bottom);
    convol_matrix = imfilter(new_matrix,H);
    resized_matrix = imresize(convol_matrix,[8,8]);
    resized_matrix = resized_matrix(resized_matrix > -100);
    stand_matrix = (resized_matrix - mean(resized_matrix))/std(double(resized_matrix));
    feature_vector(:,i) = stand_matrix;
end
    