im1=imread('1a.jpg');
im2=imread('2a.jpg');
figure;imshowpair(im1,im2,'Scaling','joint')
im1=imresize(imfilter(im1,fspecial('gaussian',7,1.),'same','replicate'),0.5,'bicubic');
im2=imresize(imfilter(im2,fspecial('gaussian',7,1.),'same','replicate'),0.5,'bicubic');

im1=im2double(im1);
im2=im2double(im2);

%figure;imshow(im1);figure;imshow(im2);

cellsize=3;
gridspacing=1;

addpath(fullfile(pwd,'mexDenseSIFT'));
addpath(fullfile(pwd,'mexDiscreteFlow'));

sift1 = mexDenseSIFT(im1,cellsize,gridspacing);
sift2 = mexDenseSIFT(im2,cellsize,gridspacing);

SIFTflowpara.alpha=2*255;
SIFTflowpara.d=40*255;
SIFTflowpara.gamma=0.005*255;
SIFTflowpara.nlevels=4;
SIFTflowpara.wsize=2;
SIFTflowpara.topwsize=10;
SIFTflowpara.nTopIterations = 60;
SIFTflowpara.nIterations= 30;


tic;[vx,vy,energylist]=SIFTflowc2f(sift1,sift2,SIFTflowpara);toc

warpI2=warpImage(im2,vx,vy);
%figure;imshow(im1);figure;imshow(warpI2);
figure;imshowpair(im1,warpI2,'Scaling','joint')

% display flow
clear flow;
flow(:,:,1)=vx;
flow(:,:,2)=vy;
figure;imshow(flowToColor(flow));

% Visualization
% BlockSize = 20;            % block size in pixels
% Master = im1;
% Slave = warpI2;
% Output = MisRegVis(Master,Slave,BlockSize);
% figure;imshow(Output);

return;

% this is the code doing the brute force matching
tic;[flow2,energylist2]=mexDiscreteFlow(sift1,sift2,[alpha,alpha*20,60,30]);toc
figure;imshow(flowToColor(flow2));

