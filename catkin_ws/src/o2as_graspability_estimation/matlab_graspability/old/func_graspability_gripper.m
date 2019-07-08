function [posx, posy, posz, rotx, roty, rotz, gscore] = func_graspability(im, partsID, binID, gripperType)
%% Fast Graspability Evaluation on a Single Depth Map 2.0
% Suitable for vacuum grippers and two-finger grippers
% 05/10/2018 Yukiyasu Domae, AIST
% 09/21/2018 latest revesion

%%%debug
    clear;
    close all;
    clc;
    partsID = 18;
    binID = 6;
    im = double(imread('1537347802432488918.tiff'));
    gripperType = "two_finger";
%     gripperType = "suction";
%%%debug

    %% parameters
    % need adjustment for each parts (shapes and sizes)
    % depends on appearance; shapes and sizes of parts
    
    % PhoXi's rotation angle(deg) on each axis(X,Y,Z)
    rot = [-11, 4, 0];

    % filter size for erode(noise reduction)
    ns = 2;
    if partsID ==8 || partsID == 16
        ns = 0;
    end

    % a threthhold for background subtraction(mm)
    bl = 3;    
    
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
            disp('Gripper Type: Two-finger');
            gT = 2;
        otherwise
            disp('ERROR; undifined gripper type');
            return;                    
    end
    
    % params which should be modified for each parts
    switch partsID
        case 4  % suction
            disp('Target: Geared motor');
            OS = 15;   % approximate size of a target parts
            R = 10;      % radius of the suction pad(pixel)    
            ow = 130;   % open width of the gripper(pixel)         
        case 5  % suction
            disp('Target: Pully for round belt');
            R = 8;
            OS = 8;
            ow = 45;
            d = 3;
        case 6 % two-finger
            disp('Target: Polyurethane round belt');
            R = 2;
            OS = 2;
            ow = 20;
        case 7  % suction
            disp('Target: Bearing with housing');
            R = 7;
            OS = 7;
            ow = 140;
        case 8 % suction & two-finger
            disp('Target: Drive shaft');
            R = 2;
            OS = 2;
            ow = 20;
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
            R = 10;
            OS = 10;
            ow = 20;
        case 12 % two-finger
            disp('Target: Bearing spacer for inner ring');
            R = 3;
            OS = 3;
            ow = 20;
        case 13
            disp('Target: Idler for round belt');
            R = 3;
            OS = 3;
            ow = 20;
        case 14
            disp('Target: Bearing shaft screw');
            R = 3;
            OS = 3;
            ow = 20;
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
            R = 3;
            OS = 3;
            ow = 20;
        case 18 % two-finger
            disp('Target: Head cap screw M3');
            R = 3;
            OS = 3;
            ow = 20;
        otherwise
            disp('ERROR; undifined parts ID')
            return;            
    end
    
%     % save a depth map
%     imwrite(im, './images/im.png', 'PNG');

    %% Settings
    % create a suctionmodel
    hm = suctionmodel(R);

    % create a two-finger gripper model
    [hm1, hm2] = twofingergrippermodel(ow, ft, fw);
    
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
    
    % image rotation 
    imrot3 = pcrot(im, rot);
    imrot3 = imrot3.*(im~=0);
    
    % reverse a rotated depth image
    img = (max(imrot3(:))-imrot3).*(imrot3~=0);

    % target image for two-finger gripper
    imgt = img.*imr;
    
    
    %% main processing
    % find positions which can be contacted by the suction gripper
    tmp = conv2(imtm,hm,'same');
    emap1 = tmp==sum(hm(:));

    % erode for noise reduction
    emap1 = imerode(emap1, ones(3,3));

    % estimated graspable objects
    emap2 = conv2(emap1,suctionmodel(OS),'same');
    
    % normal vectors
    [nx ny nz] = surfnorm(im);
    
    % graspability
    gb = emap2;

    % regionalmax
    gpeaks = imregionalmax(gb);
    gpeaks = gpeaks.*(emap1.*imtm)~=0;

%     % labeling 
%     iml = bwlabel(gpeaks);  % RGBでオーバレイ表示しよう
%     
%     % highest segments
%     imln = zeros(size(iml));
%     for ii = 1:max(iml(:))
%         tmp = (iml==ii).*img;
%         mdpth(ii) = sum(tmp(:))./(sum(sum(iml==ii)));
%         imln = imln + (iml==ii)*mdpth(ii);
%     end
    
    % find centroids from the peaks
    s = regionprops(gpeaks, 'Centroid');
    if numel(s) == 0
        disp('ERROR; no graspable points');
        return;
    end
    
    % check positions of graspable points
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
        end                
    end
    
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
    fm = mod(size(hm1,1),2);
    hs = size(hm1,1)/2;
    gg = zeros(1,numel(s));    
    gang = zeros(1,numel(s));    
    theta = [0:180/numo:180];    
    
    for kk = 1:numel(s)      
        
        if x(kk) ~=0 && y(kk)~=0
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

        else
            gg(kk) = 0;
        end
    end

    % sorting
    % [a, b] = sort(g, 'descend');
    [a, b] = sort(dpt);

    % results for a suction gripper
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

    %% show the result(vacuum)
    % vacuum gripper
    sf = 20;
    close all;
    overlaid(img,gpeaks/400),
    title('Graspable points'),    
    hold on,
    numObj = numel(s)-sum(a==0);
    for k = 1 : numObj        
        if k == 1 && gT == 1;
            plot(posx(k), posy(k), 'g*');
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
        [aa bb] = sort(gg,'descend');    
        sa2 = sum(gg==1);
        for ii = 1:sa2
            moddpt(ii) = dpt(bb(ii))        
        end
        [a b] = sort(moddpt);
        clear gscore;
        clear posx;
        clear posy;
        clear posz;
        clear rotx;
        clear roty;
        clear rotz;
        for ii = 1:sa2
                gscore(ii) = a(ii);
                posx(ii) = y(b(ii));
                posy(ii) = x(b(ii));
                posz(ii) = im(posy(ii),posx(ii)); 
                rotx(ii) = nx(posy(ii),posx(ii));
                roty(ii) = ny(posy(ii),posx(ii));
                rotz(ii) = nz(posy(ii),posx(ii));
        end
        
        %% show the results(two-finger)
        % two-finger gripper
       % numObj = sa2;
        for k = 1 : numObj

            X = x(k);
            Y = y(k);

            if gg(k)==1
                deg(k) = theta(gang(k));
                rad = deg2rad(deg(k))+pi/2;

                p = [X-ow/2, Y; X+ow/2, Y];

                if k ==  bb(b(1))
                    plot([Y-ow/2*sin(rad), Y+ow/2*sin(rad)], [X-ow/2*cos(rad), X+ow/2*cos(rad)],'g*-');
                else
                    plot([Y-ow/2*sin(rad), Y+ow/2*sin(rad)], [X-ow/2*cos(rad), X+ow/2*cos(rad)],'g.-');                    
                end
            end

        end    
    end

    hold off;
    
%     savefig('./images/result.fig');
%    openfig('./images/result.fig');

end