% test ROS connection 
%
% To use custom ROS services, please refer as below.
% https://jp.mathworks.com/help/robotics/ug/ros-custom-message-support.html#butuk98
%
% 8/6/2018
% Yukiyasu Domae, AIST

rosinit

testServer = rossvcserver('/FGE', 'graspability_estimation/CallFGE', @testROSConnection);
