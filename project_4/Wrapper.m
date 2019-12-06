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

F=DetAll{1,1};
X1 = [F(30,2), F(30,4), F(30,6), F(30,8)];
Y1 = [F(30,3), F(30,5), F(30,7),F(30,9)];
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

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior to fix first pose as origin
priorMean = Pose2(0,0,0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
graph.add(PriorFactorPose2(0, priorMean, priorNoise));
%graph.add(PriorFactorPose2(1, priorMean, priorNoise));

%% Add odometry factors
model = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
graph.add(BetweenFactorPose2(0, 1, Pose2(2, 0, 0 ), model));
graph.add(BetweenFactorPose2(1, 2, Pose2(2, 0, pi/2), model));
graph.add(BetweenFactorPose2(2, 3, Pose2(2, 0, pi/2), model));
graph.add(BetweenFactorPose2(3, 4, Pose2(2, 0, pi/2), model));