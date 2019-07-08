function resp = testROSConnection(~, req, resp)
% test a ROS service connection
% serviceType: search.srv
%
% To use custom ROS services with MATLAB, please refer as below.
% https://jp.mathworks.com/help/robotics/ug/ros-custom-message-support.html#butuk98
%
% 8/6/2018, Yukiyasu Domae, AIST

    disp('start test');
    
    close all;
    clear im;
    
    % check a req data
    disp(req.Filename); % Caution: the name is changed when MATLAB converts data.

    nam = req.Filename;
    partID = req.PartsId;
    binID = req.BinId;
    gripperType = req.GripperType;
    disp(gripperType)
    
    % input an image
    im=double(imread(nam));
    
    % graspability
    try
        [posx, posy, posz, rotx, roty, rotz, rotipz, gscore] = func_graspability(im, partID, binID, gripperType);
    catch
        resp.Success = false;
        return;
    end
    
    % params: number of the results
    if length(posx) < 1
        resp.Success = false;
        return;
    end
    
    num_results = length(posx);
    resp.ResultNum = num_results;

    % define the types of the structs
    resp.Pos3D = robotics.ros.msggen.geometry_msgs.Point;
    resp.Rot3D = robotics.ros.msggen.geometry_msgs.Point;

    pos3d = rosmessage('geometry_msgs/Point');
    rot3d = rosmessage('geometry_msgs/Point');
    for i = 1:num_results
        % result:1
        pos3d.X = posx(i);
        pos3d.Y = posy(i);
        pos3d.Z = posz(i);
        rot3d.X = rotx(i);
        rot3d.Y = roty(i);
        rot3d.Z = rotz(i);
        resp.Pos3D(i) = copy(pos3d);
        resp.Rot3D(i) = copy(rot3d);
        resp.Rotipz(i) = rotipz(i);
    end
    
    disp('end test');
end
