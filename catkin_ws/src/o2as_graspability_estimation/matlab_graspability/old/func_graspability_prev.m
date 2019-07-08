function [posx, posy, posz, rotx, roty, rotz, gscore] = func_graspability(im, partsID, binID, gripperType)
%% Fast Graspability Evaluation on a Single Depth Map 2.0
% Suitable for vacuum grippers
% 05/10/2018 Yukiyasu Domae, AIST
% 09/21/2018 latest revesion

%%%debug
    clear;
    close all;
    clc;
    partsID = 18;
    binID = 6;
    im = double(imread('1537347802432488918.tiff'));
%%%debug

    %% parameters
    % need adjustment for each parts (shapes and sizes)
    % depends on appearance; shapes and sizes of parts

    % radius of the suction pad(pixel)    
    R =   [00,00,00,10,08,02,07,02,03,03,10,03,03,03,03,03,03,03];
    %       01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18
    
    % object size
    OS = [00,00,00,15,08,02,07,02,03,03,10,03,03,03,03,03,03,03];
    %       01,02,03,04,05,06,07,08,09,10,11,12,13,14,15,16,17,18
    
    % filter size for erode(noise reduction)
    ns = 2;
    if partsID ==8
        ns = 0;
    end

    % a threthhold for background subtraction (mm)
    bl = 3;    
    
    % size of a search area for shifting graspable positions
    ml = 50;
    
    % PhoXi's rotation angle(deg) on each axis(X,Y,Z)
    rot = [-11, 4, 0];
%    rot = [0, 0, 0];
        
%     % save a depth map
%     imwrite(im, './images/im.png', 'PNG');

    %% Settings
    % create a suctionmodel
    hm = suctionmodel(R(partsID));

%     % path
%     loc = '/mnt/docker/ur-o2as/_.ros/data/20180829-PhoXiPointCloud/';
%     nam2 = 'Background_IMG_DepthMap.tif';
       
%     % background
%     imb = double(imread([loc nam2]));
% 
%     % mm->m
%     imb = imb./1000;

    % mask image ---
    imr = double(imread('imr3.png'));
    
    % select an area of the target bin
    if binID == 0
        imr = (imr~=0);
    else
        imr = (imr==binID);
    end
    
%     % background subtraction
%     imm = abs(im-imb)>(bl/1000);

    % target image
%     imt = im.*imm.*imr;
    imt = im.*imr;
    
    % noise reduction
    if ns~=0
        imtm = imerode(imt~=0,ones(ns,ns));
    else
        imtm = imt~=0;
    end
    
    %% main processing
    % find positions which can be contacted by the suction gripper
    tmp = conv2(imtm,hm,'same');
    emap1 = tmp==sum(hm(:));

    % erode for noise reduction
    emap1 = imerode(emap1, ones(3,3));

    % estimated graspable objects
    emap2 = conv2(emap1,suctionmodel(OS(partsID)),'same');
    
    % normal vectors
    [nx ny nz] = surfnorm(im);
    
    % graspability
    gb = emap2;

    % regionalmax
    gpeaks = imregionalmax(gb);
    gpeaks = gpeaks.*(emap1.*imtm)~=0;

%     % labeling 
%     L = bwlabel(gpeaks);
    
    % find centroids from the peaks
    s = regionprops(gpeaks, 'Centroid');
    if numel(s) == 0
        disp('ERROR; no graspable points');
        return;
    end
    
    
    % check positions of graspable points
%     overlaid(im,gpeaks/1000),    hold on,
    tmps = ml*ml+1;
    for kk = 1:numel(s)            
        y(kk) = round(s(kk).Centroid(1));
        x(kk) = round(s(kk).Centroid(2));
        
        if gpeaks(x(kk), y(kk)) == 0
            xmod = 0;
            ymod = 0;

            % shift a graspable point
            for ii = -ml:ml
                for jj = -ml:ml

                    if (x(kk)+ii > 0) && (x(kk)+ii < size(im,1)) && (y(kk)+jj > 0) && (y(kk)+jj < size(im,2))
                         if gpeaks(x(kk)+ii,y(kk)+jj)==1
                            if tmps >= ii^2+jj^2
                                xmod = x(kk) + ii;
                                ymod = y(kk) + jj;
                                tmps = ii^2+jj^2;
                            end
                         end
                    end
                    
                end
            end

            x(kk) = xmod;
            y(kk) = ymod;
%             plot(y(kk), x(kk), 'r.');
            
%         else
%             plot(y(kk), x(kk), 'g+');
        end        
        
    end
    
    
    % image rotation 
    imrot3 = pcrot(im, rot);
    imrot3 = imrot3.*(im~=0);
    
    % graspability on each centroids
    for kk = 1:numel(s)    
        if x(kk) ~= 0 && y(kk) ~= 0
            dpt(kk) = imrot3(x(kk),y(kk));
        else
            dpt(kk) = 0;
        end
    end

    % sorting
    % [a, b] = sort(g, 'descend');
    [a, b] = sort(dpt);

    % results
    sa = sum(a==0);
    for ii = sa+1:numel(s)
            gscore(ii-sa) = a(ii);
            posx(ii-sa) = y(b(ii));
            posy(ii-sa) = x(b(ii));
            posz(ii-sa) = im(posy(ii-sa),posx(ii-sa)); 
            rotx(ii-sa) = nx(posy(ii-sa),posx(ii-sa));
            roty(ii-sa) = ny(posy(ii-sa),posx(ii-sa));
            rotz(ii-sa) = nz(posy(ii-sa),posx(ii-sa));
    end

    %% show the result
    sf = 20;
    close all;
    overlaid(im,gpeaks/1000)
    title('Graspable points'),    
    hold on,
    numObj = numel(s)-sum(a==0);
    for k = 1 : numObj        
        if k == 1
            plot(posx(k), posy(k), 'g+');
        elseif gscore(k) > 0.8
             plot(posx(k), posy(k), 'g.');
        elseif gscore(k) > 0.6
             plot(posx(k), posy(k), 'y.');
        else
            plot(posx(k), posy(k), 'r.');
        end
    end
    hold off;
    
%     savefig('./images/result.fig');
%    openfig('./images/result.fig');

end