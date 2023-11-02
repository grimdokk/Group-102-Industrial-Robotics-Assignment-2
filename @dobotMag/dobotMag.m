classdef dobotMag < RobotBaseClass
    %% DobotMagician
    % This class is based on the DobotMagician. 
    % URL: https://en.dobot.cn/products/education/magician.html
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!
    
    % Non-constant Properties
    properties (Access = public)
        plyFileNameStem = 'dobotMag';
        currentJointAngles;
    end
    % COnstant Properties
    properties(Access = public, Constant)   
        % defaultRealQ 
        defaultRealQ  = [0 15 45 120 0] * pi/180;
        % The quantity of steps assigned to movement trajectories.
        mvmSteps = 100;
        % The duration of movements executed by the Desk Arm 5
        mvmTime = 5;
        % The maximum of manipulability necessitating the use of Damped Least Squares
        maxEpsilon = 0.1;
        % Matrix used to weight the velocity vector for movement
        mvmWeight = diag([1 1 1 0.1 0.1 0.1]);
        maxLam = 0.05;
    end

    methods (Access = public) 
%% Constructor 
        function self = dobotMag(baseTr, L)
            if nargin < 2
                % Generating a log file when one is not provided, indicating its absence
                L = log4matlab('Log_File.log');
                L.SetCommandWindowLevel(L.DEBUG);
                if nargin < 1	
            		% Recording that the default base transform has been employed
                    L.mlog = {L.DEBUG, 'dobotMag', 'The base transform is not configured. The default base transform is applied.'};
                    % Setting default base transform
                    baseTr = eye(4);
                end    
            end
            % Establishing the D&H parameter model for the Dobot Magician
            self.CreateModel();
            %Positioning the Dobot Magician within the workspace
            self.model.base = self.model.base.T * baseTr;
            % Defining the initial starting pose of the robot
            self.homeQ = self.RealQToModelQ(self.defaultRealQ);
            % Visualizing the Dobot Magician and its corresponding ply models
            self.PlotAndColourRobot();
            % Log the initialization of the Dobot Magician
            L.mlog = {L.DEBUG, 'dobotMag', 'An instance of the Dobot Magician has been instantiated within the workspace'};
        end

%% CreateModel and D&H Parameters
        function CreateModel(self)       
            link(1) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            link(2) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            link(3) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-5),deg2rad(85)]);
            link(4) = Link('d',0,        'a',0.06,      'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-180),deg2rad(180)]);
            link(5) = Link('d',-0.05,      'a',0,      'alpha',0,      'offset',pi, 'qlim',[deg2rad(-85),deg2rad(85)]);

            self.model = SerialLink(link,'name',self.name);
        end  

        %% Adjusting the Dobot Magician joint positions to reach specified angles
        function moveJoint(self, jointNo, jointVal)
            % Retrieving the current joint angles of the Dobot Magician and modifying the joint value for the specified index
            self.currentJointAngles = self.model.getpos();
            self.currentJointAngles(str2double(jointNo)) = deg2rad(jointVal);

            % Animating the robot while updating its tool transformation to manipulate the grippers.
            self.model.animate(self.currentJointAngles);
            self.updtToolTr();
            % Updating the plot
            drawnow;
        end

        %% Relocating the end-effector of the Dobot Magician to the specified Cartesian coordinates
        function moveCartes(self, coordinate)
            % Establishing the transformation for the Dobot Magician to transition to
                % Retrieving the end-effector transformation
            self.updtToolTr();
                % Obtaining the rotation matrix of the end-effector
            rotm = self.toolTr(1:3, 1:3);
            transform = [rotm coordinate'; zeros(1,3) 1];
            % Obtaining the qMatrix for moving the Dobot Magician to the specified Cartesian coordinates
            qMatrix = self.getCartesMvm(transform);
            % Iterating through the qMatrix to control the motion of the Dobot Magician
            for i = 1:size(qMatrix, 1)
                % Animating the motion of the Dobot Magician and adjusting the gripper's position
                self.model.animate(qMatrix(i,:));
                    % Modifying the end-effector's transformation
                self.updtToolTr();
                    % Updating the plot
                drawnow;
            end
        end

        %% Updating End Effector Transform
        function updtToolTr(self)
            % Updating the robot's tool transformation (toolTr) property
            % This is utilized to adjust the gripper's pose to match the end-effector
                % Retrieving the current joint angles of the Dobot Magician
            self.currentJointAngles = self.model.getpos();
                % Modifying the toolTr property
            self.toolTr = self.model.fkine(self.currentJointAngles).T;
        end

        %% Positioning the Dobot Magician to a specified transformation
        function qMatrix = getCartesMvm(self, coorTransf)
            % Determining the discrete time step
            deltT = self.mvmTime/self.mvmSteps;
            % Reserving memory for data arrays
                % Array containing measurements of manipulability
            manipl = zeros(self.mvmSteps, 1);
                % Array comprising the states of joint angles
            qMatrix = zeros(self.mvmSteps, self.model.n);
                % Array representing the velocities of the joints
            qdot = zeros(self.mvmSteps, self.model.n);
                % Array containing the angles of the end-effector
            theta = zeros(3, self.mvmSteps);
                % Array storing the x-y-z trajectory data
            traj = zeros(3, self.mvmSteps);
            
            % Retrieving the initial and final x-y-z coordinates
                % Retrieving the initial and final x-y-z coordinates
            iniTr = self.model.fkine(self.model.getpos()).T;
                % Deriving the x-y-z coordinates from the current pose transformation
            x1 = iniTr(1:3, 4);
                % Obtaining the x-y-z coordinates from the final pose transformation
            x2 = coorTransf(1:3, 4);

            % Obtaining the initial and final roll-pitch-yaw angles
                % Retrieving the initial roll-pitch-yaw angles
            rpy1 = tr2rpy(iniTr(1:3, 1:3));
                % Retrieving the final roll-pitch-yaw angles
            rpy2 = tr2rpy(coorTransf(1:3, 1:3));

            % Generating the movement trajectory
                % Generate an interpolation scalar
            s = lspb(0,1,self.mvmSteps);
            for i = 1:self.mvmSteps
                    % Generating the translation trajectory
                traj(:,i) = x1 * (1 - s(i)) + s(i) * x2;
                    % Generating the rotation trajectory
                theta(:,i) = rpy1 * (1-s(i)) + s(i) * rpy2;
            end
            % Establishing the transformation for the initial instance of the trajectory
            firstTr = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) traj(:,1); zeros(1,3) 1];
                % Obtaining the initial estimate for the joint angles
            q0 = self.model.getpos();
                % Solving the qMatrix for the initial waypoint
             qMatrix(1,:) = self.model.ikcon(firstTr, q0);

            % Following the movement trajectory using RMRC
            for i = 1:self.mvmSteps - 1
                % Obtaining the forward transformation at the current joint positions
                crrntTr = self.model.fkine(qMatrix(i,:)).T; 
                % Calculating the positional error from the upcoming waypoint
                deltX = traj(:,i+1) - crrntTr(1:3,4); 
                % Obtaining the next Roll-Pitch-Yaw (RPY) angles converted into a rotation matrix
                Rd = rpy2r(theta(1,i+1), theta(2,i+1), theta(3,i+1)); 
                % Obtaining the current end-effector's rotation matrix
                Ra = crrntTr(1:3,1:3); 
                
                % Calculating the rotation matrix for roll-pitch-yaw angular velocity
                Rdot = (1/deltT) * (Rd-Ra); 
                S = Rdot * Ra';
                % Determining the linear velocities in the x-y-z axes
                linearVel = (1/deltT) * deltX; 
                % Calculating the angular velocity for roll, pitch, and yaw
                angularVel = [S(3,2);S(1,3);S(2,1)]; 
                % Compute the end-effector matrix required to reach the next waypoint
                xdot = self.mvmWeight*[linearVel; angularVel]; 

                % Computing the Jacobian matrix for the current joint configuration
                J = self.model.jacob0(qMatrix(i,:)); 

                % Implementing Damped Least Squares
                    % Calculating the manipulability of the Dobot Magician
                manipl(i,1) = sqrt(det(J*J')); 
                    % Verifying if the manipulability falls within the specified threshold
                if manipl(i,1) < self.maxEpsilon 
                        % Damping Coefficient
                    lambda = (1 - (manipl(i,1)/self.maxEpsilon)^2) * self.maxLam; 
                    % In case Damped Least Squares (DLS) is not needed
                else 
                        % Damping Coefficient
                    lambda = 0; 
                end
                
                % DLS inverse
                invJ = inv(J'*J + lambda * eye(self.model.n))*J'; %#ok<MINV> 
                % Solving RMRC equation
                qdot(i,:) = (invJ * xdot)'; 
                % Adjusting the next joint state based on the joint velocities
                qMatrix(i+1,:) = qMatrix(i,:) + deltT * qdot(i,:); 
            end
            % Excluding motion instructions for the last link of the Dobot since the suction gripper cannot rotate
            qMatrix(:,5) = 0;
        end
    end
    
    methods(Static)
%% RealQToModelQ
        % Convert the real Q to the model Q
        function modelQ = RealQToModelQ(realQ)
            modelQ = realQ;
            modelQ(3) = DobotMagician.ComputeModelQ3GivenRealQ2and3( realQ(2), realQ(3) );
            modelQ(4) = pi - realQ(2) - modelQ(3);    
        end
        
%% ModelQ3GivenRealQ2and3
        % Convert the real Q2 & Q3 into the model Q3
        function modelQ3 = ComputeModelQ3GivenRealQ2and3(realQ2,realQ3)
            modelQ3 = pi/2 - realQ2 + realQ3;
        end
        
%% ModelQToRealQ
        % Convert the model Q to the real Q
        function realQ = ModelQToRealQ( modelQ )
            realQ = modelQ;
            realQ(3) = DobotMagician.ComputeRealQ3GivenModelQ2and3( modelQ(2), modelQ(3) );
        end
        
%% RealQ3GivenModelQ2and3
        % Convert the model Q2 & Q3 into the real Q3
        function realQ3 = ComputeRealQ3GivenModelQ2and3( modelQ2, modelQ3 )
            realQ3 = modelQ3 - pi/2 + modelQ2;
        end
    end
end