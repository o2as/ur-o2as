%% Fast Graspability Evaluation on a Single Depth Map 2.0
% Suitable for vacuum grippers
% 05/10/2018 Yukiyasu Domae, AIST

clear;
close all;
clc;

% radius of the suction pad(pixel)
R = 20;
R = 5;
R = 1;

% create a suctionmodel
hm = suctionmodel(R);

% path
% im = double(imread('sample.tif'));
loc = '/home/parallels/Documents/MATLAB/Bin/20180807-PhoXiPointCloud/';
%nam = 'BackGround_IMG_DepthMap.tif';

%nam2 = 'EmptyPart05_IMG_DepthMap.tif';
%nam1 = 'InsertedPart05_IMG_DepthMap.tif';

nam2 = 'EmptyBins_IMG_DepthMap.tif';
nam1 = 'PartsInsertedBins_IMG_DepthMap.tif';

% depthmap
im = double(imread([loc nam1]));

% background
imb = double(imread([loc nam2]));

% rectmask
% rects = getrects(im, 10);
% imr = rectmasks(im, rects);
% imwrite(imr, 'imr2.png','PNG');
imr = double(imread('imr2.png'));
% imr = ones(size(im));

% resize (if necessary)
% im = imresize(im,2);
% imb = imresize(imb,2);
% imr = imresize(imr,2);

% background subtraction
imm = abs(im-imb)>2;
imt = im.*imm.*imr;

% find positions which can be contacted by the suction gripper
tmp = conv2(double(imt~=0),hm,'same');
emap1 = tmp==sum(hm(:));

% erode for noise reduction
emap1 = imerode(emap1, ones(3,3));

% object size
OS = 20;

% estimated graspable objects
emap2 = conv2(emap1,suctionmodel(OS),'same');

% mean depth
emap3 = conv2(imt,suctionmodel(OS),'same');

% num of the results
num_results = 10;

% graspability
gb = emap2;
% gb = emap2.*emap3;

% regionalmax
gpeaks = imregionalmax(gb);

gpeaks = gpeaks.*(emap1.*(imt~=0))~=0;

% find centroids from the peaks
s = regionprops(gpeaks, 'Centroid');

% graspability on each centroids
for kk = 1:numel(s)    
    y(kk) = round(s(kk).Centroid(1));
    x(kk) = round(s(kk).Centroid(2));
    g(kk) = gb(x(kk),y(kk))./max(gb(:));
end

% sorting
[a, b] = sort(g);

% results
for ii = 1:numel(s)
    gscore(ii) = a(numel(s)-ii+1);
    posy(ii) = y(b(numel(s)-ii+1));
    posx(ii) = x(b(numel(s)-ii+1));
end

% show the result
sf = 20;
imv(im),
title('Graspable points'),
hold on,
numObj = numel(s);
for k = 1 : numObj
    txt = num2str(round(gscore(k),3));
    txt = num2str(b(k));
    plot(posy(k), posx(k), 'g.');
   % text(posy(k)+sf, posx(k), txt, 'color', 'green');            
end
