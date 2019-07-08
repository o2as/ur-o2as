% extract RoIs from an image by mouse click
% 
% im: input image 
% num_RoI: the number of RoIs which are extracted
%
% imm: masked image of RoIs
%
% 08/08/2018 Yukiyasu Domae, AIST
% 09/19/2018 latest revision

%% params
% the number of RoIs
num_RoI = 10;

% path
loc = '/mnt/docker/share/';
nam = 'Initial_IMG_DepthMap.tif';

%% initialization
% target image
im = double(imread([loc nam]));

% normalize
im = uint8(255.*im./max(im(:)));

% target image
imm = zeros(size(im));

%% main process
% extract RoIs
for ii = 1:num_RoI
    ii
    if ii == 1
       figure(1),
       movegui(1, [0, 0]);
    end
    tmp = roipoly(im);
    movegui(1, [0, 0]);
    imm = imm + tmp*ii;
end

% % normalize
% imm = imm~=0;
    
% save the image
imwrite(uint8(imm), '/home/osx/ur-o2as/catkin_ws/src/o2as_graspability_estimation/graspability_estimation/data/images/imr3.png', 'PNG');