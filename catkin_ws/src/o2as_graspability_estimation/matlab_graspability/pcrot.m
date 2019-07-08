function imr = pcrot(im, rot)
%% depth image rotation
% 3d rotation of a single depth image by affine transform
%
% input;
% im: target depth image
% rot: rotation angle(deg) on each axis(X,Y,Z)
%  e.g. rot = [-4, 7, 0]
%
% output;
% imr: rotated depth image
%
% 09/19/2018 Yukiyasu Domae, AIST

% rotation matrix
tx = deg2rad(rot(1));
ty = deg2rad(rot(2));
tz = deg2rad(rot(3));
Rx = [1 0 0 0; 0 cos(tx) -sin(tx) 0; 0 sin(tx) cos(tx) 0; 0 0 0 1];
Ry = [cos(ty) 0 sin(ty) 0; 0 1 0 0; -sin(ty) 0 cos(ty) 0; 0 0 0 1];
Rz = [cos(tz) -sin(tz) 0 0; sin(tz) cos(tz) 0 0; 0 0 1 0; 0 0 0 1];
Rt = affine3d(Rz*Ry*Rx);

% depth image -> pointcloud
matx = repmat(1:size(im,2),size(im,1),1);
maty = repmat(1:size(im,1),size(im,2),1)';
xyz(:,:,1) = matx;
xyz(:,:,2) = maty;
xyz(:,:,3) = im*1000; % m->mm
ptc = pointCloud(xyz);

% rotation
ptcr = pctransform(ptc,Rt);

% pointcloud -> depth image
imr = ptcr.Location(:,:,3)./1000; % mm->m

% % visualize
% figure, pcshow(ptc);
% figure, pcshow(ptcr)
