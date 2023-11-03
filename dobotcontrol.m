%% Real Dobot Manipulation Demo Code
rosinit('localhost');

jointTargetHigh = [-1.4031, 0.1965, 0.2839, 0];
jointTargetLow = [-1.9732, 1.0798, 0.9234, 0];
jointTargetThird = [-0.802, 1.1441, 0.8229, 0];

%% Getting the current safety status of the robot
safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;

%% Loop for Dobot Movement
while true
   

    
    %% Setting suction cup
    % Turn on the tool
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [1]; %#ok<NBRAK2> % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);
    pause;

    %% Moving Dobot Upwards
    jointTarget = jointTargetHigh; % Remember that the Dobot has 4 joints by default.
    
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    
    send(targetJointTrajPub,targetJointTrajMsg);
    pause;

    %% Moving Dobot Down
    jointTarget = jointTargetThird; % Remember that the Dobot has 4 joints by default.
    
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    
    send(targetJointTrajPub,targetJointTrajMsg);
    pause;
    
    %% Moving Dobot Upwards
    jointTarget = jointTargetHigh; % Remember that the Dobot has 4 joints by default.
    
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    
    send(targetJointTrajPub,targetJointTrajMsg);
    pause;

    

    %% Setting suction cup
    % Turn on the tool
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [0]; %#ok<NBRAK2> % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);
    pause;  

end