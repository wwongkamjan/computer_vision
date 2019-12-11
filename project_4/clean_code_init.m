%% Wrapper for the CMSC426Course final project at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
%          (Originally for CMSC828T)
% 
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
%load('DataSquareNew.mat');
load('DataMappingNew.mat');
load('CalibParams.mat');

%% SLAM Using GTSAM
%[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                %IMU, LeftImgs, TLeftImgs);
                                                
%% Estimating pose for origin (R)
pos = 19;
F1 = DetAll{1};
X1 = [F1(pos,2), F1(pos,4), F1(pos,6), F1(pos,8)];
Y1 = [F1(pos,3), F1(pos,5), F1(pos,7),F1(pos,9)];

tags =[];
for foo=1:length(DetAll)
    mat = DetAll{foo}(:, 1);
    tags = [tags;mat];
    tags = unique(tags);
end

XR = [0,TagSize, TagSize, 0];
YR = [0,0, TagSize, TagSize];

% XR = [0,TagSize,TagSize, 0];
% YR = [TagSize,TagSize, 0 ,0];

H = est_homography(XR,YR,X1,Y1);
H = H/H(3,3);

camPose = cell(1,length(DetAll));

DetAllReal = cell(1,length(DetAll));
TryDetAllReal = [];
temp = [];
for j = 1:length(DetAll{1})
    IC = DetAll{1};
    curr = IC(j,:); 
    [x1,y1] = apply_homography(H, curr(2), curr(3));
    [x2,y2] = apply_homography(H, curr(4), curr(5));
    [x3,y3]= apply_homography(H, curr(6), curr(7));
    [x4,y4] = apply_homography(H, curr(8), curr(9));

    if IC(j,1) ~= 10
        temp1 = [IC(j,1) x1 y1 1 x2 y2 1 x3 y3 1 x4 y4 1];
    else
        temp1 = [10 0 0 1 TagSize 0 1 TagSize TagSize 1 0 TagSize 1];
    end 
    temp = [temp; temp1];
end
DetAllReal{1}= temp;

TryDetAllReal = [TryDetAllReal; temp];
%% Estimate camera pose for rest of the tags

K_graph = cameraParameters('Intrinsicmatrix', K');
for i= 2:length(DetAll)
    temp = [];
    %F1 = DetAllReal{i-1} ;
    set1 = TryDetAllReal(:,1);
    F2 = DetAll{1,i};
    set2 = F2(:,1);
    CommonTag = intersect(set1,set2);
    X1 = []; Y1 = []; XR = []; YR = [];
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
    H = H/H(3,3);
    imagePoints =[];
    worldPoints = [];
    for j = 2:size(DetAll{i}, 1)
        IC = DetAll{i};
        if ismember(IC(j,1), CommonTag)
           set1 = TryDetAllReal(:,1);
           ind = find(set1 == IC(j,1));
           x = IC(j, :);
           imagePoints = [imagePoints; x(2) x(3); x(4) x(5); x(6) x(7); x(8) x(9)];
           tag = TryDetAllReal(ind,:);
           temp1 = [IC(j,1) tag(2) tag(3) 1 tag(5) tag(6) 1 tag(8) tag(9) 1 tag(11) tag(12) 1];
           worldPoints = [worldPoints; tag(2) tag(3) 1; tag(5) tag(6) 1; tag(8) tag(9) 1; tag(11) tag(12) 1];
           
        else
           curr = IC(j,:);
           [x1,y1] = apply_homography(H, curr(2), curr(3));
           [x2,y2] = apply_homography(H, curr(4), curr(5));
           [x3,y3]= apply_homography(H, curr(6), curr(7));
           [x4,y4] = apply_homography(H, curr(8), curr(9));
           temp1 = [IC(j,1) x1 y1 1 x2 y2 1 x3 y3 1 x4 y4 1];
        end
        temp = [temp; temp1];
  
        if ~ismember(TryDetAllReal(:,1), IC(j,1))
            TryDetAllReal = [TryDetAllReal; temp1];
        end
    
    end
    
    [worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,...
                                        worldPoints, K_graph, 'Confidence', 5);
         plot3(worldLocation(1), worldLocation(2), worldLocation(3), 'blacko', 'DisplayName', 'Poses')
         hold on;
    DetAllReal{i}= temp;
    campose{i} = [worldOrientation worldLocation];
end

%% estimate world camera pose
%  for i = 1:length(camPose)
%     %plot3(abs(camPose{1, i}(1,4)), abs(camPose{1, i}(2,4)), abs(camPose{1, i}(3,4)), 'blacko', 'DisplayName', 'Poses')
%     plot(camPose{1, i}(1,4), camPose{1, i}(2,4), 'blacko', 'DisplayName', 'Poses')
%     hold on;
%  end

%  plotCamera('Size',10,'Orientation',worldOrientation,'Location',...
%      worldLocation);
%  hold off

%% Plot Camera Poses for all Frames
%AllPoses = zeros(length(DetAll), 7);

% for i = 1:length(DetAll)
%     loc = -camPose{1, i}(:,1:3)' * camPose{1, i}(:,4);
%     AllPoses(i, :) = [loc',  rotm2quat(camPose{1, i}(:,1:3))];
% end
 %plot3(AllPoses(:, 1), AllPoses(:, 2), AllPoses(:, 3), 'blacko', 'DisplayName', 'Poses')
% %plot(AllPoses(:, 1), AllPoses(:, 2), 'blacko', 'DisplayName', 'Poses')
 %hold on;

% for i = 2:length(camPose)
%     quat = rotm2quat(camPose{1, i}(:,1:3));
%     plotTransforms(camPose{1, i}(:,4)', quat)
%     hold on;
% end
   
%  figure;
% for i = 1:length(camPose)
%     orientation = camPose{1, i}(:,1:3)';
%     location = -camPose{1, i}(:,4)' * orientation;
%     cam = plotCamera(location,orientation);%,'Size',20);
%     hold on;
% end
%% Plot Real World Coordinates for Tags
%figure;
    for j = 1:length(TryDetAllReal(:,1))%length(DetAllReal)
        temp = TryDetAllReal;
        %for j = 1:length(temp)
            x1 = [temp(j, 2), temp(j, 5), temp(j, 8), temp(j, 11)];
            y1 = [temp(j, 3), temp(j, 6), temp(j, 9), temp(j, 12)];
            if temp(j,1) == 10
                scatter(x1,y1,"o");
            else
                scatter(x1,y1,"*");
            end
            hold on;
        %end
    end

%% Creating the factor graph
% graph = NonlinearFactorGraph;
% measurementNoiseSigma = 1.0;
% pointNoiseSigma = [0.1 0.1 0.1]';
% poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
% poseNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
% K1 = Cal3_S2(K(1, 1), K(2, 2), 0, K(1,3), K(2, 3));
% graph.add(PriorFactorPose3(symbol('x',1), Pose3(Rot3(camPose{1,1}(:,1:3)), Point3(camPose{1,1}(:,4))), poseNoise));
% graph.add(PriorFactorPoint3(symbol('l', 10), Point3(0, 0, 0), pointNoise));
% graph.add(PriorFactorPoint3(symbol('m', 10), Point3(0, TagSize, 0), pointNoise));
% graph.add(PriorFactorPoint3(symbol('n', 10), Point3(TagSize, TagSize, 0), pointNoise));
% graph.add(PriorFactorPoint3(symbol('o', 10), Point3(TagSize,0, 0), pointNoise));
% 
% for i=1:length(DetAll)
%     mat = DetAll{i};
%     
%     if i < length(DetAll)
%         pose1 = Pose3(Rot3(camPose{1, i}(:,1:3)), Point3(camPose{1, i}(:,4)));
%         pose2 = Pose3(Rot3(camPose{1, i+1}(:,1:3)), Point3(camPose{1, i}(:,4)));
%         relPose = pose1.between(pose2);
%         graph.add(BetweenFactorPose3(symbol('x', i), symbol('x', i+1), relPose, poseNoise));
%     end
%     
%     for k=1:size(mat, 1)
%         dat = mat(k, :);
%         graph.add(GenericProjectionFactorCal3_S2(Point2(dat(2), dat(3)), measurementNoise, symbol('x',i), symbol('l',dat(1)), K1));
%         graph.add(GenericProjectionFactorCal3_S2(Point2(dat(4), dat(5)), measurementNoise, symbol('x',i), symbol('m',dat(1)), K1));
%         graph.add(GenericProjectionFactorCal3_S2(Point2(dat(6), dat(7)), measurementNoise, symbol('x',i), symbol('n',dat(1)), K1));
%         graph.add(GenericProjectionFactorCal3_S2(Point2(dat(8), dat(9)), measurementNoise, symbol('x',i), symbol('o',dat(1)), K1));
%     end
% end
% 
% % % Optimize the graph
% sprintf('\nGraph made\n')
% 
% optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);