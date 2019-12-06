%% Wrapper for the CMSC426Course final project at University of Maryland, College Park
% Code by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
%          (Originally for CMSC828T)

clc
clear all
close all
import gtsam.*

%% Add ToolBox to Path eg. ToolboxPath = 'gtsam_toolbox';
ToolboxPath = 'gtsam_toolbox';
addpath(ToolboxPath);


%% Load Data
% Download data from the following link: 
% https://drive.google.com/open?id=1ZFXZEv4yWgaVDE1JD6-oYL2KQDypnEUU
load('DataSquareNew.mat');
load('CalibParams.mat');

%% SLAM Using GTSAM
%[LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
%                                                IMU, LeftImgs, TLeftImgs);


%XRYR1 = inv(K)*inv(H)*[X1(1) Y1(1)]

%% estimating pose

F1=DetAll{1,1};
X1 = [F1(30,2), F1(30,4), F1(30,6), F1(30,8)];
Y1 = [F1(30,3), F1(30,5), F1(30,7),F1(30,9)];
%plot(X1,Y1)

XR = [0,0+TagSize,0+TagSize,0];
YR = [0,0,0-TagSize,0-TagSize];
%plot(XR,YR)

H = est_homography(XR,YR,X1,Y1);
temp = inv(K)*H;
h1 = temp(:,1);
h2 = temp(:,2);
h3 = temp(:,3);

KH = [h1 h2 h1.*h2];
R0 = svd(KH)
T0 = h3/(norm(h1))
x0 = [R0 T0]
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
    temp1 = [IC(j,1) x1 y1 x2 y2 x3 y3 x4 y4 ];
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
    h1 = homo(:,1);
    h2 = homo(:,2);
    h3 = homo(:,3);

    KH = [h1 h2 h1.*h2];
    R = svd(KH);
    T = h3/(norm(h1));
    x = [R T];
    camPose{1,i} = x;
    
    for j = 1:length(DetAll{1,i})
        IC = DetAll{1,i};
        curr = IC(j,:); 
        [x1,y1] = apply_homography(H, curr(2), curr(3));
        [x2,y2] = apply_homography(H, curr(4), curr(5));
        [x3,y3]= apply_homography(H, curr(6), curr(7));
        [x4,y4] = apply_homography(H, curr(8), curr(9));
        temp1 = [IC(j,1) x1 y1 x2 y2 x3 y3 x4 y4 ];
        temp = [temp; temp1];
        
    end
    DetAllReal{1,i}= temp;
end








%% Create graph container and add factors to it
% graph = NonlinearFactorGraph;
% 
% %% Add prior to fix first pose as origin
% priorMean = Pose2(0,0,0); % prior at origin
% priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
% graph.add(PriorFactorPose2(0, priorMean, priorNoise));
% %graph.add(PriorFactorPose2(1, priorMean, priorNoise));
% 
% %% Add odometry factors
% model = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
% graph.add(BetweenFactorPose2(0, 1, Pose2(2, 0, 0 ), model));
% graph.add(BetweenFactorPose2(1, 2, Pose2(2, 0, pi/2), model));
% graph.add(BetweenFactorPose2(2, 3, Pose2(2, 0, pi/2), model));
% graph.add(BetweenFactorPose2(3, 4, Pose2(2, 0, pi/2), model));