% read an input 
%imds = imageDatastore({'1.jpg','2.jpg', '3.jpg'});
%imds = imageDatastore({'Images/Set1/1.jpg', 'Images/Set1/2.jpg', 'Images/Set1/3.jpg'});
%imds = imageDatastore({'Images/Set2/1.jpg', 'Images/Set2/2.jpg', 'Images/Set2/3.jpg'});
imds = imageDatastore({'Images/Set3/1.jpg', 'Images/Set3/2.jpg', 'Images/Set3/3.jpg'});%,  'Images/Set3/4.jpg', 'Images/Set3/5.jpg', 'Images/Set3/6.jpg', 'Images/Set3/7.jpg', 'Images/Set3/8.jpg'});
scale = 0.6;
num_images = numel(imds.Files);
train_raw = cell(num_images);
for i = 1:num_images
    train_raw{i} = readimage(imds,i);
end

for i = 1:num_images
    train{i} = imresize(train_raw{i}, scale);
end

imageSize = zeros(num_images,2);
for i = 1:num_images
    train_gray{i} = rgb2gray(train{i});
    imageSize(i,:) = size(train_gray{i});
    corners{i} = cornermetric(train_gray{i});
end

Nbest = 200;
ratio = 0.5;
n_max = 100;


%i = 1;
for i = 1:num_images
    [x_best{i}, y_best{i}, cbest{i}] = ANMS(corners{i}, Nbest);
end

%[x_best_1_1, y_best_1_1, cbest1_1] = ANMS(cm1_1, Nbest);
%[x_best_1_2, y_best_1_2, cbest1_2] = ANMS(cm1_2, Nbest);
%[x_best_1_3, y_best_1_3, cbest1_3] = ANMS(cm1_3, Nbest);

% for i = 1:num_images
%     figure;
%     imshow(train{i});
%     hold on;
%     scatter(y_best{i}, x_best{i}, 'r.')
%     hold off;
% end

% imshow(train1_2);
% hold on;
% scatter(y_best_1_2, x_best_1_2, 'r.')
% hold off;
% 
% imshow(train1_3);
% hold on;
% scatter(y_best_1_3, x_best_1_3, 'r.')
% hold off;

for i = 1:num_images
    feature_vector{i} = featureDescriptor(train_gray{i}, x_best{i}, y_best{i});
end
% feature_vector1_1 = featureDescriptor(train1_1, x_best_1_1, y_best_1_1);
% feature_vector1_2 = featureDescriptor(train1_2, x_best_1_2, y_best_1_2);
% feature_vector1_3 = featureDescriptor(train1_3, x_best_1_3, y_best_1_3);

tforms(num_images) = projective2d(eye(3));
for i = 2:num_images
  [matched_points{i - 1}, matched_points{i}] = featureMatching(cbest{i - 1}, cbest{i}, feature_vector{i - 1}, feature_vector{i}, ratio); 
  [inliers{i - 1}, inliers{i}, H] = RANSAC(matched_points{i - 1}, matched_points{i}, n_max);
  figure;
  hold on;
  showMatchedFeatures(train{i -1}, train{i}, inliers{i-1}, inliers{i}, 'montage');
  hold off;
  tforms(i) = projective2d(inv(H)');
  %tforms(i) = estimateGeometricTransform(inliers{i}, inliers{i - 1},'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);
  tforms(i).T = tforms(i).T*tforms(i - 1).T;
end
% [matched_points1, matched_points2] = featureMatching(cbest1_1, cbest1_2, feature_vector1_1, feature_vector1_2, ratio);
% [matched_points1, matched_points3] = featureMatching(cbest1_1, cbest1_3, feature_vector1_1, feature_vector1_3, ratio);

%figure; 
%ax = axes;
%showMatchedFeatures(train1_1, train1_3, matched_points1, matched_points3, 'montage');
%showMatchedFeatures(train1_1, train1_2, matched_points1, matched_points2, 'montage');
%title(ax, 'Feature Matching');
%legend(ax, 'Matched points 1','Matched points 2');

%[match_1, match_2, H] = RANSAC(matched_points1, matched_points2, threshold, n_max);
%[match_1, match_3, H] = RANSAC(matched_points1, matched_points3, threshold, n_max);
% figure; 
% ax = axes;
% showMatchedFeatures(train1_1, train1_2, match_1, match_2, 'montage');%, 'Parent', ax);
%showMatchedFeatures(train1_1, train1_3, match_1, match_3, 'montage');
% title(ax, 'RANSAC');
%legend(ax, 'Matched points 1','Matched points 2');



for i = 1:numel(tforms)       
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

%maxImageSize = max(size(train1_1), size(train1_2))
maxImageSize = max(imageSize);

% Find the minimum and maximum output limits 
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);

yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);

% Width and height of panorama.
width  = round(xMax - xMin);
height = round(yMax - yMin);

% Initialize the "empty" panorama.
panorama = zeros([height width 3], 'like', train{num_images});


blender = vision.AlphaBlender('Operation', 'Binary mask', 'MaskSource','Input port');

% Create a 2-D spatial reference object defining the size of the panorama.
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], double(xLimits), double(yLimits));

for i = 1:num_images
    
    I = readimage(imds, i);   
    I = imresize(I, scale);
    % Transform I into the panorama.
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
                  
    % Generate a binary mask.    
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    
    % Overlay the warpedImage onto the panorama.
    panorama = step(blender, panorama, warpedImage, mask);
end

figure
imshow(panorama)

function [x_best,y_best, c_best] = ANMS(cm,Nbest)
lm = imregionalmax(cm);
[x,y] = find(lm);
N_Strong = size(x,1);
r = inf(1,N_Strong);
ED = 0;

for i = 1:N_Strong
    for j = 1:N_Strong
        if cm(x(j),y(j)) > cm(x(i),y(i))
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
end


function feature_vector = featureDescriptor(train, x_best, y_best)
H = fspecial('gaussian', 40);
train = padarray(train, [20 20], 0, 'both');
feature_vector = zeros(64, size(x_best, 2));

for i = 1:size(x_best, 2)
    new_matrix = train(x_best(i):x_best(i) + 40, y_best(i):y_best(i) + 40);
    new_matrix = im2double(new_matrix);
    convol_matrix = imfilter(new_matrix, H, 'conv');
    resized_matrix = imresize(convol_matrix, [8, 8]);
    resized_vector = double(resized_matrix(resized_matrix < inf));
    feature_vector(:,i) = (resized_vector - mean(resized_vector))/std(resized_vector);
end
end

function [matched_points1, matched_points2] = featureMatching(I1, I2, feature_vector1, feature_vector2, ratio)
k = 1;
f1 = feature_vector1;
f2 = feature_vector2;
for i = 1:size(feature_vector1, 2)
    smallest = inf;
    second_smallest = inf;
    for j = 1:size(feature_vector2, 2)
        d = norm(feature_vector1(:, i) - feature_vector2(:, j))^2;
        if(d < smallest)
            second_smallest = smallest;
            smallest = d;
            index = j;
        elseif (d < second_smallest && d ~= smallest)
            second_smallest = d;
        end
    end
    if smallest/second_smallest < ratio
        matched_points1(k, :) = [I1(2, i), I1(1, i)];
        matched_points2(k, :) = [I2(2, index), I2(1, index)];
        feature_vector1(:,i) = [];
        feature_vector2(:, index) = [];
        k = k + 1;
    end
end
end

function [match_1, match_2, H] = RANSAC(matched_points1, matched_points2, n_max)
X = zeros(4,1);
Y = zeros(4,1);
x = zeros(4,1);
y = zeros(4,1);
percentage = 0;
j = 0;

while j <= n_max && percentage < 70
    for i = 1:4
        r = randi(size(matched_points1, 1));
        X(i, 1) = matched_points1(r, 2); %source
        Y(i, 1) = matched_points1(r, 1);
        x(i, 1) = matched_points2(r, 2); %dest
        y(i, 1) = matched_points2(r, 1);
    end
    
    H = est_homography(x, y, X, Y);
    [x1, y1] = apply_homography(H, matched_points1(:, 1), matched_points1(:, 2));    
    SSD(:, 1) = matched_points2(:, 1) - x1;
    SSD(:, 2) = matched_points2(:, 2) - y1;
    SSD_n = SSD(:, 1) + SSD(:, 2);
    SSD_n = SSD_n.^2;
    threshold = 10^floor(log10(mean(SSD_n))/2.5);
    if size(SSD_n(SSD_n < threshold), 1)/size(SSD_n, 1) * 100 > percentage 
        percentage = size(SSD_n(SSD_n < threshold), 1) / size(SSD_n, 1) * 100;
        inliers = find(SSD_n < threshold);
    end
    j = j + 1;
end

inliers_1 = matched_points1(inliers, :);
inliers_2 = matched_points2(inliers, :);

H = est_homography(inliers_2(:, 1), inliers_2(:, 2), inliers_1(:, 1), inliers_1(:, 2));
[X2, Y2] = apply_homography(H, inliers_1(:, 1), inliers_1(:, 2));  
match_1 = inliers_1;
match_2 = inliers_2;
%match_2 = [X2 Y2];
%[X2, Y2] = apply_homography(H,inliers_2(:, 1), inliers_2(:, 2));  
%match_2 = [X2 Y2];
end

