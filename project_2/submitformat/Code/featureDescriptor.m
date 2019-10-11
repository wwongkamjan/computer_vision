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