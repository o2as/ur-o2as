function [posx, posy, posz, rotx, roty, rotz, rotipz, gscore] = func_graspability(im, partsID, binID, gripperType)
%% Fast Graspability Evaluation on a Single Depth Map 2.0
% Suitable for vacuum grippers and two-finger grippers
% 05/10/2018 Yukiyasu Domae, AIST
% 10/07/2018 latest revesion

    close all;
    noimr = 0;
    
% %% debug
% % ���ӁF�{�Ԃ̋����摜�̒P�ʂ�m
%     clear;
%     close all;
%     clc;
%     noimr = 0;
%     %
%     partsID = 4;
%     binID = 1; %8;  % "0" is OK. "0" means that all bins become targets.
%     %
% % %     im = double(imread('1537347802432488918.tiff'));
% %     im = double(imread('1538282183173146963.tiff'));
% %     im = double(imread('.\two-finger\001.tif'));
% %     im = double(imread('.\holes\001.tif'))/1000;
% %     im = double(imread('.\velocity-0.1\set_1_part_4.tiff'));
% %     im = double(imread('.\20181004-Picking\20181004-004.tiff'));
% %     im = double(imread('.\20181004-HardAngledWorkPose\Parts_8_IMG_DepthMap.tif'));
%     im = double(imread('.\OSX\sample.tif'))/1000;      
% %     im = double(imread('.\20181007_osx.tif'))/1000;      
%     %
% %     gripperType = "inner";
%     gripperType = "two_finger";
% %     gripperType = "suction";

    %% parameters
    % need adjustment for each parts (shapes and sizes)
    % depends on appearance; shapes and sizes of parts
    
    % PhoXi's rotation angle(deg) on each axis(X,Y,Z)
    rot = [-10.9, 3.6, 0];

    % for debug: calibration after setting 3D sensor
%     imrot3 = pcrot(im, rot);
%     imrot3 = imrot3.*(im~=0);
%     close all;
%     figure, mesh(imrot3*1000), axis equal;
%     img = (max(imrot3(:))-imrot3).*(imrot3~=0);
%     imv2(img.*(img>0.349), 0.3, 0.4)    % bin 1-5, 7-10
%     imv2(img.*(img>0.343), 0.3, 0.4)    % bin 6
%     imv2(img.*(img>0.335), 0.3, 0.4)    % bin 5 **

    % filter size for erode(noise reduction)
    ns = 2;
    if partsID ==8 || partsID == 16 || partsID == 14 || partsID == 17 || partsID == 18
        ns = 0;
    end
    
    % size of a search area for shifting graspable positions
    ml = 50;
    
    % finger thickness for a two finger gripper(pixel)
    ft = 1;

    % finger width for a two finger gripper(pixel)
    fw = 5;
    
    % the number of oriantation of a two-finger gripper
    numo = 8;    
    
    % depth for insertion of a two-finger gripper(mm)
    d = 1;

    % type of a gripper
    switch gripperType
        case "suction"
            disp('Gripper type: Suction');
            gT = 1;
        case "two_finger"
            disp('Gripper type: Two-finger');
            gT = 2;
        case "inner"
            disp('Gripper type: Two-finger(inner)');
            gT = 3;
        otherwise
            disp('ERROR; undifined gripper type');
            return;                    
    end
    
    % params which should be modified for each parts
    switch partsID
        case 4  % suction
            disp('Target: Geared motor');
            OS = 10;   % approximate size of a target parts
            R = 6;      % radius of the suction pad(pixel)    
            ow = 30;   % open width of the gripper(pixel)         
            d = 1;
        case 5  % suction
            disp('Target: Pully for round belt');
            R = 6;
            OS = 8;
            ow = 45;
            d = 3;
        case 6 % two-finger
            disp('Target: Polyurethane round belt');
            R = 2;
            OS = 2;
            ow = 20;
            d = 5;
        case 7  % suction   % x: height, o: segment size
            disp('Target: Bearing with housing');
            R = 12; %7;
            OS = 12; %7;
            ow = 50;
        case 8 % suction & two-finger
            disp('Target: Drive shaft');
            R = 2;
            OS = 4;
            ow = 20;%22;
            d = 4;
        case 9
            disp('Target: End cap for shaft');
            R = 3;
            OS = 3;
            ow = 20;
        case 10
            disp('Target: Bearing spacers for inner ring');
            R = 3;
            OS = 3;
            ow = 20;
        case 11
            disp('Target: Pully for round belts clamping');
            R = 8;
            OS = 8;
            ow = 20;
        case 12 % two-finger % x: height, o:
            disp('Target: Bearing spacer for inner ring');
            R = 2;
            OS = 4;
            ow = 20;
        case 13
            disp('Target: Idler for round belt');
            R = 5;
            OS = 7;
            ow = 30;
        case 14
            disp('Target: Bearing shaft screw');
            R = 2;
            OS = 4; % too small, results detect from noises
            ow = 14;
            d = 5;
        case 15
            disp('Target: Hex nut');
            R = 3;
            OS = 3;
            ow = 20;
        case 16 % two-finger
            disp('Target: Flat washer');
            R = 3;
            OS = 3;
            ow = 15;
        case 17 % two-finger
            disp('Target: Head cap screw M4');
            R = 1;
            OS = 1;
            ow = 10;
            d = 1;
        case 18 % two-finger
            disp('Target: Head cap screw M3');
            R = 1;
            OS = 1;
            ow = 10;
            d = 1;
        otherwise
            disp('ERROR; undifined parts ID')
            return;            
    end
    
    if gT == 3
        R = 8; % radius of the two-finger gripper
    end
    
    % minimum radius of detected circles (pixel)
    minr = 8;
    
    % maximum radius of detected circles (pixel)
    maxr = 30;
        
    %% Settings
    % create a suctionmodel
    hm = suctionmodel(R);

    % create a two-finger gripper model
    [hm1, hm2] = twofingergrippermodel(ow, ft, fw);
        
    % mask image
    imr = double(imread('imr3.png'));    
    if noimr == 1
        imr =binID*ones(size(im));
    end
    
    % select an area of the target bin
    if binID == 0
        imr = (imr~=0);
    else
        imr = (imr==binID);
    end
    
    % target image
    imt = im.*imr;
    
    % noise reduction
    if ns~=0
        imtm = imerode(imt~=0,ones(ns,ns));
    else
        imtm = imt~=0;
    end
    
    % image rotation 
    imrot3 = pcrot(im, rot);
    imrot3 = imrot3.*(im~=0);
    
    % reverse a rotated depth image
    img = (max(imrot3(:))-imrot3).*(imrot3~=0);

    % target image for two-finger gripper
    imgt = img.*imr;
    imgt = imgt~=0;
    
    % edge detection from normal vectors
    if partsID ~= 8 && partsID ~= 14
        [~, ~, nmz] = surfnorm(imrot3*1000);    
        imz = nmz > 0.9;    
        imgt = (imgt.*imz)~=0;
    end
           
    % normal vectors
    [nx, ny, nz] = surfnorm(im*1000);

    %% main processing
    %% main process: suction and two-finger
    if gT == 1 || gT == 2

        % find positions which can be contacted by the suction gripper
        tmp = conv2(imgt,hm,'same');
        emap1 = tmp==sum(hm(:));

        % erode for noise reduction
        emap1 = imerode(emap1, ones(3,3));
        
        % estimated graspable objects
        emap2 = conv2(emap1,suctionmodel(OS),'same');

        % graspability
        gb = emap2;

        % find regional max values
        gpeaks = imregionalmax(gb);
        gpeaks = gpeaks.*(emap1.*imtm)~=0;

        % find centroids from the peaks
        s = regionprops(gpeaks, 'Centroid');
        if numel(s) == 0
            disp('ERROR; no graspable points');
            return;
        end

        % if centroids are holes, shift suction points
        x = zeros(numel(s),1);
        y = zeros(numel(s),1);
        for kk = 1:numel(s)            
            tmps = ml*ml+1;
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
            end                
        end

        dpt = zeros(numel(s), 1);
        dpt2 = zeros(numel(s), 1);        
        % depth on each centroids
        for kk = 1:numel(s)    
            if x(kk) ~= 0 && y(kk) ~= 0
                dpt(kk) = imrot3(x(kk),y(kk));
                dpt2(kk) = img(x(kk),y(kk));
            else
                dpt(kk) = 0;
                dpt2(kk) = 0;
            end
        end        

        % collision check for the gripper   
%         fm = mod(size(hm1,1),2);
        hs = size(hm1,1)/2;
        gg = zeros(1,numel(s));    
        gang = zeros(1,numel(s));    
        theta = 0:180/numo:180;    

        % size check
        for ii = 1:numo-1
           hmo2 = imrotate(hm2,theta(ii));
           tmp = size(hmo2,1)/2;
           if tmp > hs
               hs = tmp;
           end
        end
        hsb = hs;

        for kk = 1:numel(s)      
            if x(kk) ~= 0 && y(kk) ~= 0 && x(kk)-hsb > 0 && x(kk)+hsb < size(im,1) && y(kk)-hsb > 0 && y(kk)+hsb < size(im,2) % 20180926 revise
               for ii = 1:numo-1
                   hmo2 = imrotate(hm2,theta(ii));
                   hs = size(hmo2,1)/2;
                   tmp = img(x(kk)-hs:x(kk)+hs-1, y(kk)-hs:y(kk)+hs-1);
                   cm = tmp >= dpt2(kk)-d/1000;
                   tmp2 = cm.*hmo2;
                   if sum(tmp2(:))==0
                       gg(kk) = 1;
                       gang(kk) = ii;
                   end
               end
            end 
        end
        
        % sort results
        % [a, b] = sort(g, 'descend');
        [a, b] = sort(dpt);

        % results for a suction gripper
        sa = sum(a==0);
        gscore = zeros(numel(s)-sa,1);
        posx = zeros(numel(s)-sa,1);
        posy = zeros(numel(s)-sa,1);
        posz = zeros(numel(s)-sa,1);
        rotx = zeros(numel(s)-sa,1);
        roty = zeros(numel(s)-sa,1);
        rotz = zeros(numel(s)-sa,1);
        for ii = sa+1:numel(s)
            gscore(ii-sa) = a(ii);
            posx(ii-sa) = y(b(ii));
            posy(ii-sa) = x(b(ii));
            posz(ii-sa) = im(posy(ii-sa),posx(ii-sa)); 
            rotx(ii-sa) = nx(posy(ii-sa),posx(ii-sa));
            roty(ii-sa) = ny(posy(ii-sa),posx(ii-sa));
            rotz(ii-sa) = nz(posy(ii-sa),posx(ii-sa));
        end
        
        % 'descent' gscore 20181007 -domae
        gscore = 2-gscore./min(gscore);

        % find main orientation on each point % 20181007 -domae
        L = bwlabel(emap1);
        rotipz = zeros(numel(posx),1);
        for kk = 1:numel(posx)
            tmpl = L(posy(kk), posx(kk));
            tmp = (L==tmpl);
            rpo = regionprops(tmp, 'orientation');
            rotipz(kk) = rpo.Orientation;
        end
        
    %% main process: inner picking    
    elseif gT == 3
        
        % find holes for two-finger(inner)
        [centers, radii, ~] = imfindcircles(imgt, [minr maxr]);   

        % check collisions around the holes
        hs = size(hm,1)/2;
        gg = zeros(1,size(centers,1));
        radm = mean(radii);
        dpt = zeros(size(centers,1),1);
        mtmp = zeros(size(centers,1),1);
        for ii = 1:size(centers,1)
            % mean depth
            md = 0;
            smt = 0;
            for jj = 1:100
                tmprot = 2*pi/100*jj;
                tmp = img(round(centers(ii,2)+cos(tmprot)*(radm)), round(centers(ii,1)+sin(tmprot)*(radm)));
                md = md + tmp;
                if tmp == 0 
                    smt = smt + 1;
                end                
            end
            
            dpt(ii) = md/(100-smt+eps);
            
            % hole depth            
           tmp = img(round(centers(ii,2))-hs:round(centers(ii,2))+hs-1, round(centers(ii,1))-hs:round(centers(ii,1))+hs-1).*hm;
           tmp2 = (tmp == 0);
           mtmp(ii) = sum(tmp(:))/(size(tmp,1)*size(tmp,2)-sum(tmp2(:))+eps);           
           if dpt(ii) <= mtmp(ii)
               dpt(ii) = 0;
           end
        end
        
        % for debug
        dptm = dpt;
        figure, hold on, plot(dptm, 'r+:'), plot(mtmp, '.:'), hold off;
        
        % normalize
        dpt = dpt./max(dpt(:));
        
        % sorting
        % [a, b] = sort(g, 'descend');
        [a, b] = sort(dpt, 'descend');

        % results for a two-finger(inner) gripper
        sa = sum(a==0);
        gscore = zeros(numel(a)-sa,1);
        posx = zeros(numel(a)-sa,1);
        posy = zeros(numel(a)-sa,1);
        posz = zeros(numel(a)-sa,1);
        rotx = zeros(numel(a)-sa,1);
        roty = zeros(numel(a)-sa,1);
        rotz = zeros(numel(a)-sa,1);        
        rotipz = zeros(numel(a)-sa,1);        
        for ii = 1:numel(a)-sa
            gscore(ii) = a(ii);
            posx(ii) = round(centers(b(ii),1));
            posy(ii) = round(centers(b(ii),2));
            posz(ii) = dpt(b(ii)); 
            rotx(ii) = nx(posy(ii),posx(ii));
            roty(ii) = ny(posy(ii),posx(ii));
            rotz(ii) = nz(posy(ii),posx(ii));
            rotipz(ii) = 0;
        end        
    end  

    %% show the result(vacuum)
    % vacuum gripper
    if gT == 1 || gT == 2
        overlaid(img,gpeaks/600),
        numObj = numel(s)-sum(a==0);
    elseif gT == 3
        imv(img),
%         imv2(img, 840, 890),
%         imv2(img, 1980, 2040),
        for ii = 1:size(centers,1)
            if dpt(ii) ~= 0
                viscircles(centers(ii,:), radm, 'EdgeColor', 'green'),
            end
        end
        numObj = size(a,1)-sa;
    end
    title('Graspable points'),
    hold on,
    for k = 1 : numObj        
        
        rad = deg2rad(rotipz(k));
        plot([posx(k)-ow/2*cos(rad), posx(k)+ow/2*cos(rad)], [posy(k)+ow/2*sin(rad), posy(k)-ow/2*sin(rad)],'b-');            
        
        if k == 1 && (gT == 1 || gT == 3)
            plot(posx(k), posy(k), 'g*');
        elseif k == 2 && (gT == 1 || gT == 3)
            plot(posx(k), posy(k), 'g+');
        elseif k == 3 && (gT == 1 || gT == 3)
            plot(posx(k), posy(k), 'g+');            
        elseif gscore(k) > 0.8
             plot(posx(k), posy(k), 'g.');
        elseif gscore(k) > 0.6
             plot(posx(k), posy(k), 'y.');
        else
            plot(posx(k), posy(k), 'r.');
        end
        
    end
    
    %% two-finger
    % results for a two-finger gripper    
    if gT == 2
        [~, bb] = sort(gg,'descend');    
        sa2 = sum(gg==1);
        moddpt = zeros(sa2,1);
        if sa2~=0
            for ii = 1:sa2
                moddpt(ii) = dpt(bb(ii));     
            end
            clear a
            clear b
            [a, b] = sort(moddpt);
            clear gscore;
            clear posx;
            clear posy;
            clear posz;
            clear rotx;
            clear roty;
            clear rotz;
            for ii = 1:sa2
                if y(b(ii)) ~= 0 && x(b(ii)) ~= 0
                    gscore(ii) = a(ii);
                    posx(ii) = y(b(ii));
                    posy(ii) = x(b(ii));
                    posz(ii) = im(posy(ii),posx(ii)); 
                    rotx(ii) = nx(posy(ii),posx(ii));
                    roty(ii) = ny(posy(ii),posx(ii));
                    rotz(ii) = nz(posy(ii),posx(ii));
                    rotipz(ii) = theta(gang(bb(ii)));
                end
            end
            
            % 'descent' gscore 20181007 -domae
            gscore = 2-gscore./min(gscore);

            %% show the results(two-finger)
            % two-finger gripper
           ii = 1;
           deg = zeros(numObj,1);
            for k = 1 : numObj
                X = x(k);
                Y = y(k);               
                if gg(k)==1
                    deg(k) = theta(gang(k));
                    rad = deg2rad(deg(k))+pi/2;                    
                    ii = ii + 1;
                    if k ==  bb(b(1))
                        plot([Y-ow/2*sin(rad), Y+ow/2*sin(rad)], [X-ow/2*cos(rad), X+ow/2*cos(rad)],'g*-');
                    else
                        plot([Y-ow/2*sin(rad), Y+ow/2*sin(rad)], [X-ow/2*cos(rad), X+ow/2*cos(rad)],'g.-');                    
                    end
                end
            end
        end    
    end
    hold off;
    
end