% Test func_graspability standalone
%
% 10/3/2018 Yuma Hijioka, AIST


im = './old/1537347802432488918.tiff';
partsID = 8; % '4'--'18' is allowed. If set others, func_graspability may be occured any errors.
binID = 0; % "0" is OK. "0" means that all bins become targets.
gripperType = 'suction';
% gripperType = 'two_finger'; % two_finger means two_finger gripper approaches outside the target.
% gripperType = 'inner'; % inner means two_finger gripper approaches inside the target. (enable to use when the work has whole.)

disp("Graspability evaluation started...")
[posx, posy, posz, rotx, roty, rotz, rotipz, gscore] = func_graspability(im, partsID, binID, gripperType)
disp("Finshed!")

disp("The best result: ")
disp(posx(1))
disp(posy(1))
disp(posz(1))
disp(rotX(1))
disp(rotY(1))
disp(rotz(1))
disp(rotipz(1))
disp(gscore(1))

