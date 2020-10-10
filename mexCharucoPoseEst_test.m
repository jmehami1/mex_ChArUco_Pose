%Test the CharucoPoseEst mex function

%Author: Jasprabhjit Mehami, 13446277

clear;
close all;
%
% rosshutdown;
% rosinit;
%
% %subscribers
% subImagePS = rossubscriber("/camera/rgb/image_raw");

% 
% if exist("build", "dir")
%     addpath("build");
% else
%     error("mex file not built");
% end

%test image
img = imread("patexample.png");

%camera parameters
intrMat = [532.568131996427,0,0;0,531.905416600879,0;327.499527166381,231.227840418968,1]; %intrinsic matrix for OPENCV
distRad = [0.0346875042867809,-0.0917743770901257,-0.0897944587524139];
distTan = [-0.00415109739624088,0.00571543700759848];
distCoefCV = [distRad(1:2), distTan, distRad(3)]; %array of distortion coefficients in opencv

figure('Name', 'Before passing in');
imshow(img);

%mex function for getting pose of the pattern
[rotMat, trans, found, imgOut] = CharucoPosEst(img, intrMat, distCoefCV, 8, 6, 0.04, 0.03);


figure('Name', 'After passing in');
imshow(imgOut);


 disp(isequal(img, imgOut));