function [inliers_1, inliers_2, H] = RANSAC(matched_points1, matched_points2, n_max)
X = zeros(4,1);
Y = zeros(4,1);
x = zeros(4,1);
y = zeros(4,1);
percentage = 0;
j = 0;

while j <= n_max && percentage < 70
    for i = 1:4
        r = randi(size(matched_points1, 1));
        X(i, 1) = matched_points1(r, 2); %XY source pi
        Y(i, 1) = matched_points1(r, 1);
        x(i, 1) = matched_points2(r, 2); %xy dest pi dash
        y(i, 1) = matched_points2(r, 1);
    end
    
    H = est_homography(x, y, X, Y);
    [x1, y1] = apply_homography(H, matched_points1(:, 1), matched_points1(:, 2));    
    SSD(:, 1) = matched_points2(:, 1) - x1;
    SSD(:, 2) = matched_points2(:, 2) - y1;
    SSD_n = SSD(:, 1) + SSD(:, 2);
    SSD_n = SSD_n.^2;
    threshold = 10^floor(log10(mean(SSD_n))/2.5);
    %threshold = 10^floor(log10(mean(SSD_n))/2.6);
    if size(SSD_n(SSD_n < threshold), 1)/size(SSD_n, 1) * 100 > percentage 
        percentage = size(SSD_n(SSD_n < threshold), 1) / size(SSD_n, 1) * 100;
        inliers = find(SSD_n < threshold);
    end
    j = j + 1;
end

inliers_1 = matched_points1(inliers, :);
inliers_2 = matched_points2(inliers, :);
H = est_homography(inliers_2(:, 2), inliers_2(:, 1), inliers_1(:, 2), inliers_1(:, 1));

end