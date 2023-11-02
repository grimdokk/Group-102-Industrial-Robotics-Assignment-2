%% The 2F-85 Robotiq Gripper for the Aubo i5.
% https://robotiq.com/products/2f85-140-adaptive-robot-gripper

classdef Grippers < RobotBaseClass
    %% Gripper Class Properties
    % Non-constant Properties
    properties (Access = public)
        % A variable that determines whether the gripper finger is currently in an open or closed state
        stat = false; 
        % The stem name utilized to discover associated ply files during the plotting process
        plyFileNameStem = 'Gripper'; 
    end

    % Constant Properties
    properties (Access = public, Constant)
        % The initial or default pose for the gripper finger
        iniJointAng = [0 45 45]*pi/180; 
        % The joint angles required to close the gripper fingers
        clsJointAng = [0 92.5 4]*pi/180; 
        % The default number of steps for a movement
        mvmSteps = 100; 
    end

    %% ...structors
    methods
        %% Constructor for 2F-85
        function self = Grippers(baseTr,fingerNum,L)
            % Establishing the default base transformation in case it has not been specified within the constructor inputs
            if nargin < 3
                % Generating a log file if one is not provided, indicating that it has not been created yet
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 2
                    % Recording that the default base transformation has been applied
                    L.mlog = {L.DEBUG,'Gripper','The base transformation has not been configured. The default base transformation is applied.'};
                        % Assigning the default base transformation
                    baseTr = eye(4); 
                end

                if nargin < 1
                    % Recording the usage of the default finger number
                    L.mlog = {L.DEBUG,'Gripper','The finger number has not been specified. The default finger number is used.'};
                        % Assigning the default finger number.
                    fingerNum = 1; 
                end
            end
            
            % Constructing the D&H parameter model for the 2F-85 gripper
            self.crtModel(fingerNum); 

            % Positioning the 2F-85 gripper within the workspace.
            self.model.base = self.model.base.T * baseTr;
                % Configuring the initial pose of the Aubo i5.
            self.homeQ = self.iniJointAng; 

            % Rotating the second finger by 180 degrees.
            if fingerNum == 2
                self.model.base = self.model.base.T * trotz(pi);
            end

            % Visualizing the 2F-85 gripper and its corresponding PLY models.
            self.PlotAndColourRobot();

            % Recording the creation of the gripper finger
            L.mlog = {L.DEBUG,'Gripper',['Gripper Finger ',num2str(fingerNum),' created within the workspace']};
            
            % Logging the establishment of the 2F-85 gripper
            if fingerNum == 2
                L.mlog = {L.DEBUG,'Gripper','2F-85 gripper object created within the workspace'};
            end
        end

        %% Serial link creation with D&H parameters
        function crtModel(self, fingerNum)
            % D&H parameters for the 2F-85 finger model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % https://robotiq.com/products/2f85-140-adaptive-robot-gripper
            link(1) = Link([0    0.06118   0.01270   pi/2   0   0]);
            link(2) = Link([0    0         0.05718   0      0   0]);
            link(3) = Link([0    0         0.04510   0      0   0]);

            % Incorporating joint limits for both finger types, with identical limits
            link(1).qlim = [0 0];
            link(2).qlim = [45 95]*pi/180;
            link(3).qlim = [-2 45]*pi/180;

            % Constructing the serial link object
            self.model = SerialLink(link,'name',[self.name, num2str(fingerNum)]);
        end

        %% A method for retrieving the QMatrix to open/close gripper fingers
        function qMatrix = GetOpenCloseQMatrix(self)
            % Switch statement to modify the qMatrix generated
            % Determined by whether the gripper is open or closed
            switch(self.stat)
                % The gripper is presently open and requires closing
                case false 
                    qMatrix = jtraj(self.iniJointAng, self.clsJointAng, self.mvmSteps);
                    self.stat = true;
                % The gripper is currently closed and needs to be opened
                case true 
                    qMatrix = jtraj(self.clsJointAng, self.iniJointAng, self.mvmSteps);
                    self.stat = false;
            end
        end

        %% Position updater for the gripper
        function updtGripperPos(self, baseTr, fingerNum)
            % Adjusting the base position of the gripper finger
            self.model.base = baseTr;

            % Rotating the second finger by 180 degrees
            if fingerNum == 2
                self.model.base = self.model.base.T * trotz(pi);
            end

            % Animating the changing pose of the gripper
            self.model.animate(self.model.getpos());
        end

    end
end
