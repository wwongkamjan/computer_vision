clc
clear all
close all
import gtsam.*
%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
ToolboxPath = 'D:\UMCP\Courses\CMSC426\P4_CV\gtsam_toolbox';
addpath(ToolboxPath);

%% Load Data
% Download data from the following link: 
% https://drive.google.com/open?id=1ZFXZEv4yWgaVDE1JD6-oYL2KQDypnEUU
load('DataSquare.mat');
%load('DataMappingNew.mat');
load('CalibParams.mat');

%% SLAM Using GTSAM
%[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                %IMU, LeftImgs, TLeftImgs);
                                                
%% Estimating pose for origin (R)
F1 = DetAll{1};

X1 = [F1(30,2), F1(30,4), F1(30,6), F1(30,8)];
Y1 = [F1(30,3), F1(30,5), F1(30,7),F1(30,9)];

%plot(X1,Y1)

XR = [0,TagSize,TagSize,0];
YR = [0,0,TagSize,TagSize];

% XR = [0,0,TagSize,TagSize];
% YR = [0,TagSize,TagSize,0];

% XR = [0,TagSize,TagSize,0];
% YR = [TagSize,TagSize,0,0];

H = est_homography(XR,YR,X1,Y1);
temp = inv(K)*H;
h1 = temp(:,1);
h1 = h1/norm(h1);
h2 = temp(:,2);
h2 = h2/norm(h2);
h3 = temp(:,3);
KH = [h1 h2 cross(h1,h2)];
T0 = h3/norm(h1);
% T0 = T0/max(T0);
%norm(h1)
% h1 = temp(1,:);
% h2 = temp(2,:);
% h3 = temp(3,:);
% KH = [h1' h2' cross(h1',h2')];
%T0 = h3'/(norm(h1'));

[U, Sigma, V] = svd(KH);
Sigma = [1 0 0; 0 1 0; 0 0 det(U*V')];
R0 = U*Sigma*V';

camPose = cell(1,length(DetAll));
camPose{1} = [R0 T0];

DetAllReal = cell(1,length(DetAll));
TryDetAllReal = [];
temp = [];
for j = 1:length(DetAll{1})
    IC = DetAll{1};
    curr = IC(j,:); 
    [x1,y1] = apply_homography(H, curr(2), curr(3));
    [x2,y2] = apply_homography(H, curr(4), curr(5));
    [x3,y3] = apply_homography(H, curr(6), curr(7));
    [x4,y4] = apply_homography(H, curr(8), curr(9));

    if IC(j,1) ~= 10
        temp1 = [IC(j,1) x1 y1 0 x2 y2 0 x3 y3 0 x4 y4 0];
    else
        temp1 = [10 0 0 0 TagSize 0 0 TagSize TagSize 0 0 TagSize 0];
    end 
    temp = [temp; temp1];
end
DetAllReal{1}= temp;

TryDetAllReal = [TryDetAllReal; temp];
%% Estimate camera pose for rest of the tags
for i= 2:length(DetAll)
    temp = [];
    %F1 = DetAllReal{i-1} ;
    set1 = TryDetAllReal(:,1);
    F2 = DetAll{1,i};
    set2 = F2(:,1);
    CommonTag = intersect(set1,set2);
    X1 = [];
    Y1 = [];
    XR = [];
    YR = [];
    for k = 1:length(CommonTag)
        ind = find(set2 == CommonTag(k));
        tag = F2(ind,:);
        X = [tag(2) tag(4) tag(6) tag(8)];
        Y = [tag(3) tag(5) tag(7) tag(9)];
        X1 = [X1 X];
        Y1 = [Y1 Y];
        
        ind = find(set1== CommonTag(k));
        tag = TryDetAllReal(ind, :);
        X = [tag(2) tag(5) tag(8) tag(11)];
        Y = [tag(3) tag(6) tag(9) tag(12)];
        XR = [XR X];
        YR = [YR Y];        
    end

    H = est_homography(XR,YR,X1,Y1);
    homo = inv(K)*H;
    h1 = homo(:,1); 
    h1 = h1/norm(h1)
    h2 = homo(:,2); h3 = homo(:,3);
    h2 = h2/norm(h2)
    KH = [h1 h2 cross(h1,h2)];
    T = h3/norm(h1);
%     T = T/max(T);
    
    
%   h1 = homo(1,:); h2 = homo(2,:); h3 = homo(3,:); 
%   KH = [h1' h2' cross(h1',h2')];
%   T = h3'/(norm(h1'));
    
    [U, Sigma, V] = svd(KH);
    Sigma = [1 0 0; 0 1 0; 0 0 det(U*V')];
    R = U*Sigma*V';
    camPose{1,i} = [R T];
    
    for j = 2:length(DetAll{i})
        IC = DetAll{i};
        if ismember(IC(j,1),CommonTag)
           %F1 = DetAllReal{i-1};
           %F1 = TryDetAllReal()
           set1 = TryDetAllReal(:,1);
           ind = find(set1 == IC(j,1));
           tag = TryDetAllReal(ind,:);
           temp1 = [IC(j,1) tag(2) tag(3) 0 tag(4) tag(5) 0 tag(6) tag(7) 0 tag(8) tag(9) 0];
        else
           curr = IC(j,:);
           [x1,y1] = apply_homography(H, curr(2), curr(3));
           [x2,y2] = apply_homography(H, curr(4), curr(5));
           [x3,y3]= apply_homography(H, curr(6), curr(7));
           [x4,y4] = apply_homography(H, curr(8), curr(9));
           temp1 = [IC(j,1) x1 y1 0 x2 y2 0 x3 y3 0 x4 y4 0];
        end
        %temp = [temp; temp1];
        if ~ismember(TryDetAllReal(:,1), IC(j,1))
            TryDetAllReal = [TryDetAllReal; temp1];
        end
        
    end
    DetAllReal{i}= temp;
    
end

%% Plot Camera Poses for all Frames
for i = 1:length(camPose)
    quat = rotm2quat(camPose{1, i}(:,1:3));
    plotTransforms(camPose{1, i}(:,4)', quat)
    hold on;
end
    
hold off; 


graph = NonlinearFactorGraph;
measurementNoiseSigma = 1.0;
pointNoiseSigma = [0.1 0.1 0.1]';
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
poseNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
pointNoise = noiseModel.Diagonal.Sigmas(pointNoiseSigma);
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
K1 = Cal3_S2(K(1, 1), K(2, 2), 0, K(1,3), K(2, 3));
initialEstimate = Values;

for i=1:length(DetAll)
    mat = DetAll{i};
    
    if i < length(DetAll)
        pose1 = Pose3(Rot3(camPose{1, i}(:,1:3)), Point3(camPose{1, i}(:,4)));
        pose2 = Pose3(Rot3(camPose{1, i+1}(:,1:3)), Point3(camPose{1, i+1}(:,4)));
        relPose = pose1.between(pose2);
        graph.add(BetweenFactorPose3(symbol('x', i), symbol('x', i+1), relPose, poseNoise));
    end
    
    for k=1:size(mat, 1)
        dat = mat(k, :);
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(2), dat(3)), measurementNoise, symbol('x',i), symbol('l',dat(1)), K1));
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(4), dat(5)), measurementNoise, symbol('x',i), symbol('m',dat(1)), K1));
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(6), dat(7)), measurementNoise, symbol('x',i), symbol('n',dat(1)), K1));
        graph.add(GenericProjectionFactorCal3_S2(Point2(dat(8), dat(9)), measurementNoise, symbol('x',i), symbol('o',dat(1)), K1));
    end
end

graph.add(PriorFactorPose3(symbol('x',1), Pose3(Rot3(camPose{1,1}(:,1:3)), Point3(camPose{1,1}(:,4))), poseNoise));
graph.add(PriorFactorPoint3(symbol('l', 10), Point3(0, 0, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('m', 10), Point3(0, TagSize, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('n', 10), Point3(TagSize, TagSize, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('o', 10), Point3(0, TagSize, 0), pointNoise));

for idx = 1:length(DetAll)
    initialEstimate.insert(symbol('x', idx), Pose3(Rot3(camPose{1,idx}(:,1:3)), Point3(camPose{1,idx}(:,4))));
end
temp = TryDetAllReal;
for iter = 1:length(TryDetAllReal(:,1))
    
    graph.add(BetweenFactorPoint3(symbol('l', temp(iter, 1)), symbol('m', temp(iter, 1)), Point3(TagSize, 0, 0), pointNoise));
    graph.add(BetweenFactorPoint3(symbol('m', temp(iter, 1)), symbol('n', temp(iter, 1)), Point3(0, TagSize, 0), pointNoise));
    graph.add(BetweenFactorPoint3(symbol('o', temp(iter, 1)), symbol('n', temp(iter, 1)), Point3(TagSize, 0, 0), pointNoise));
    graph.add(BetweenFactorPoint3(symbol('l', temp(iter, 1)), symbol('o', temp(iter, 1)), Point3(0, TagSize, 0), pointNoise));
    
    initialEstimate.insert(symbol('l',	temp(iter, 1)),Point3(temp(iter, 2:4)'));
    initialEstimate.insert(symbol('m',temp(iter, 1)),Point3(temp(iter, 5:7)'));
    initialEstimate.insert(symbol('n',temp(iter, 1)),Point3(temp(iter, 8:10)'));
    initialEstimate.insert(symbol('o',temp(iter, 1)),Point3(temp(iter, 11:13)'));
end



initialEstimate.print(sprintf('\nInitial estimate:\n'));
% % Optimize the graph
sprintf('\nGraph made\n')

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);

result = optimizer.optimize();
result.print(sprintf('\nFinal result:\n  '));

% marginals = Marginals(graph, result);
% cla
% hold on;
% 
% plot3DPoints(result, []);
% plot3DTrajectory(result, '*', 1, 8);

tags = TryDetAllReal(:,1);
% Retrieve landmarks from graph
for id = 1:length(TryDetAllReal(:,1))
    pl(id, :) = [result.at(symbol('l', tags(id))).x result.at(symbol('l', tags(id))).y result.at(symbol('l', tags(id))).z];
    pm(id, :) = [result.at(symbol('m', tags(id))).x result.at(symbol('m', tags(id))).y result.at(symbol('m', tags(id))).z];
    pn(id, :) = [result.at(symbol('n', tags(id))).x result.at(symbol('n', tags(id))).y result.at(symbol('n', tags(id))).z];
    po(id, :) = [result.at(symbol('o', tags(id))).x result.at(symbol('o', tags(id))).y result.at(symbol('o', tags(id))).z];
end

LandMarksComputed = [tags pl pm pn po];
[~,idx] = sort(LandMarksComputed(:,1)); % sort just the first column
LandMarksComputed = LandMarksComputed(idx,:);

% Retrieve poses from graph
AllPoses = zeros(length(DetAll), 7);

for bar = 1:length(DetAll)
    pose = result.at(symbol('x', bar));
    rotm = pose.rotation.matrix;
    tr = pose.translation.vector;
    loc = -rotm' * tr;
    AllPoses(bar, :) = [loc',  rotm2quat(rotm)];
end

% Remove inconsistent poses

AllPosesComputed(1, :) = AllPoses(1, :);  

for pntr = 2:length(DetAll)
    point1 = AllPoses(pntr, 1:3);
    point0 = AllPosesComputed(size(AllPosesComputed, 1), 1:3);
    distV = norm(point0 - point1);
%     distV
%     if distV < 200000
        AllPosesComputed(size(AllPosesComputed, 1)+1, :) = AllPoses(pntr, :);
%     end
end

cla
plot(LandMarksComputed(:, 2), LandMarksComputed(:, 3), 'r*')
hold on
plot(LandMarksComputed(:, 5), LandMarksComputed(:, 6), 'b*')
hold on
plot(LandMarksComputed(:, 8), LandMarksComputed(:, 9), 'cyan*')
hold on
plot(LandMarksComputed(:, 11), LandMarksComputed(:, 12), 'green*')
hold on
plot(AllPosesComputed(:, 1), AllPosesComputed(:, 2), 'blacko')
hold on
legend('show')

% means = mean(AllPoses2(:, 1:3));
% sd = std(AllPoses2(:, 1:3));
% 
% t1 = abs(means+3*sd);
% t2 = abs(means-3*sd);
% 
% for qq = 1:size(AllPoses2, 1)
%     pose1 = AllPoses2(qq, 1:3);
%     if all(abs(pose1) < t1) && all(abs(pose1) > t2)
%         AllPosesComputed(qq, :) = pose1;
%     end
% end
% 
% AllPosesComputed = AllPosesComputed(any(AllPosesComputed, 2), :);
% 
% plot3(AllPosesComputed(:, 1), AllPosesComputed(:, 2), AllPosesComputed(:, 3))
% 
% plot3(points(:, 2), points(:, 3), points(:, 4), 'r*')
% hold on
% plot3(points(:, 5), points(:, 6), points(:, 7), 'b*')
% hold on
% plot3(points(:, 8), points(:, 9), points(:, 10), 'black*')
% hold on
% plot3(points(:, 11), points(:, 12), points(:, 13), 'green*')
