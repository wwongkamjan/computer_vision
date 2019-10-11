function [matched_points1, matched_points2] = featureMatching(I1, I2, feature_vector1, feature_vector2, ratio)
k = 1;
%f1 = feature_vector1;
%f2 = feature_vector2;
for i = 1:size(feature_vector1, 2)
    smallest = inf;
    second_smallest = inf;
    for j = 1:size(feature_vector2, 2)
        if(feature_vector2(:, j) == -inf)
            continue
        end
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
        %feature_vector1(:,i) = -inf;
        feature_vector2(:, index) = -inf;
        k = k + 1;
    end
end
end