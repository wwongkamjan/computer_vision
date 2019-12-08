%% Wrapper for the CMSC426Course final project at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
%          (Originally for CMSC828T)
% 
% clc
% clear all
% close all
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
                                                
%%Estimating pose for origin (R)
F1=DetAll{1,1};
X1 = [F1(30,2), F1(30,4), F1(30,6), F1(30,8)];
Y1 = [F1(30,3), F1(30,5), F1(30,7),F1(30,9)];
%plot(X1,Y1)

% XR = [0,TagSize,TagSize,0];
% YR = [0,0,-TagSize,-TagSize];

XR = [0,0,TagSize,TagSize];
YR = [0,TagSize,TagSize,0];

% XR = [0,TagSize,TagSize,0];
% YR = [0,0,TagSize,TagSize];

H = est_homography(XR,YR,X1,Y1);

temp = inv(K)*H;
%temp = H;
% h1 = temp(:,1);
% h2 = temp(:,2);
% h3 = temp(:,3);
% KH = [h1 h2 cross(h1,h2)];
h1 = temp(1,:);
h2 = temp(2,:);
h3 = temp(3,:);

KH = [h1' h2' cross(h1',h2')];

[U, Sigma, V] = svd(KH);
Sigma = [1 0 0; 0 1 0; 0 0 det(U*V')];
R0 = U*Sigma*V';
%R0 = KH;
T0 = h3'/(norm(h1'));
%T0 = h3/(norm(h1));
new = [1 1 1];
x0 = [R0 T0];
%loc = -R0' * T0
%pose = [loc,R0]

camPose = cell(1,length(DetAll));
camPose{1,1} = x0;


DetAllReal = cell(1,length(DetAll));
temp = [];
for j = 1:length(DetAll{1,1})
    IC = DetAll{1,1};
    curr = IC(j,:); 
    [x1,y1] = apply_homography(H, curr(2), curr(3));
    [x2,y2] = apply_homography(H, curr(4), curr(5));
    [x3,y3]= apply_homography(H, curr(6), curr(7));
    [x4,y4] = apply_homography(H, curr(8), curr(9));
    if IC(j,1) ~= 10
        temp1 = [IC(j,1) x1 y1 x2 y2 x3 y3 x4 y4 ];
    else
        temp1 = [10 0 0 0 TagSize TagSize TagSize TagSize 0 ];
    end 
    temp = [temp; temp1];

end
DetAllReal{1,1}= temp;

for i= 2:length(DetAll)
    temp = [];
    F1 = DetAllReal{1,i-1} ;
    set1 = F1(:,1);
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
        tag = F1(ind, :);
        X = [tag(2) tag(4) tag(6) tag(8)];
        Y = [tag(3) tag(5) tag(7) tag(9)];
        XR = [XR X];
        YR = [YR Y];        
    end

    H = est_homography(XR,YR,X1,Y1);
    homo = inv(K)*H;
%     h1 = homo(:,1);
%     h2 = homo(:,2);
%     h3 = homo(:,3);
    h1 = homo(1,:);
    h2 = homo(2,:);
    h3 = homo(3,:);
    KH = [h1' h2' cross(h1',h2')];
    %KH = [h1 h2 cross(h1,h2)];
    [U, Sigma, V] = svd(KH);
    Sigma = [1 0 0; 0 1 0; 0 0 det(U*V')];
    R = U*Sigma*V';
    %T = h3/(norm(h1));
    T = h3'/(norm(h1'));
    new = [1 1 1];
    x = [R T];
    camPose{1,i} = x;
    
    for j = 2:length(DetAll{1,i})
        IC = DetAll{1,i};
        if ismember(IC(j,1),CommonTag)
           F1 = DetAllReal{1,i-1};
           set1 = F1(:,1);
           ind = find(set1 == IC(j,1));
           tag = F1(ind,:);
           x1 = tag(2);
           x2 = tag(4);
           x3 = tag(6);
           x4 = tag(8);
           y1 = tag(3);
           y2 = tag(5);
           y3 = tag(7);
           y4 = tag(9);
           temp1 = [IC(j,1) x1 y1 x2 y2 x3 y3 x4 y4];
        else
           curr = IC(j,:);
           [x1,y1] = apply_homography(H, curr(2), curr(3));
           [x2,y2] = apply_homography(H, curr(4), curr(5));
           [x3,y3]= apply_homography(H, curr(6), curr(7));
           [x4,y4] = apply_homography(H, curr(8), curr(9));
           temp1 = [IC(j,1) x1 y1 x2 y2 x3 y3 x4 y4];
        end
        temp = [temp; temp1];
        
    end
    DetAllReal{1,i}= temp;
end
    for i = 1:length(camPose)
        %hold on;
%         plot3(camPose{1, i}(:,1:3),camPose{1, i}(:,4), [1;1;1])
        quat = rotm2quat(camPose{1, i}(:,1:3));
        plotTransforms(camPose{1, i}(:,4)', quat);
        hold on;
    end
graph = NonlinearFactorGraph;
measurementNoiseSigma = 1.0;
pointNoiseSigma = [0.1 0.1 0.1]';
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';
poseNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
K1 = Cal3_S2(K(1, 1), K(2, 2), 0, K(1,3), K(2, 3));
graph.add(PriorFactorPose3(symbol('x',1), Pose3(Rot3(camPose{1,1}(:,1:3)), Point3(camPose{1,1}(:,4))), poseNoise));
graph.add(PriorFactorPoint3(symbol('l', 10), Point3(0, 0, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('m', 10), Point3(0, TagSize, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('n', 10), Point3(TagSize, TagSize, 0), pointNoise));
graph.add(PriorFactorPoint3(symbol('o', 10), Point3(TagSize,0, 0), pointNoise));

for i=1:length(DetAll)
    mat = DetAll{i};
    
    if i < length(DetAll)
        pose1 = Pose3(Rot3(camPose{1, i}(:,1:3)), Point3(camPose{1, i}(:,4)));
        pose2 = Pose3(Rot3(camPose{1, i+1}(:,1:3)), Point3(camPose{1, i}(:,4)));
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

% % Optimize the graph
sprintf('\nGraph made\n')

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
