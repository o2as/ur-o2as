function [hm1, hm2] = twofingergrippermodel(ow, ft, fw);
% create a twofinger gripper model
%
% input;
% ow: open width of the gripper
% ft: finger thickness 
% fw: finger width
%
% output;
% hm1: contact model
% hm2: collision model
%

    W = max(ow+2*ft, fw)+2;
    hm1 = zeros(W,W);
    hm2 = zeros(W,W);

    c = round(W/2);
    how = round(ow/2);
    hft = round(ft/2);
    hfw = round(fw/2);
        
    hm1(c-hft:c+hft, c-how:c+how) = 1;

    hm2(c-hfw:c+hfw, c-how-hft:c-how) = 1;
    hm2(c-hfw:c+hfw, c+how:c+how+hft) = 1;
 
end
