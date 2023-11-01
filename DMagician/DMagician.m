%% Dobot Magician Robot
% URL: https://en.dobot.cn/products/education/magician.html

classdef DMagician < RobotBaseClass
    %% Robot Class Properties
    % Non-constant properties
    properties (Access = public)   
        plyFileNameStem = 'DMagician'; % Name stem used to find associated ply model files
    end

    % Constant properties
    properties (Access = public, Constant)
        defaultRealQ  = [0 15 45 120 0]*pi/180; % Default joint angle

        movementSteps = 1000; % Number of steps allocated for movement trajectories
        movementTime = 5; % Time for movements undergone by the Aubo i5
        epsilon = 0.1; % Maximum measure of manipulability to then require Damped Least Squares
        movementWeight = diag([1 1 1 1 1 1]); % Weighting matrix for movement velocity vector
        maxLambda = 0.05;
    end

    %% ...structors
    methods (Access = public) 
        %% Constructor 
        function self = DMagician(baseTr, L)
			% Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % % Logging that the default base transform has been used
                    % L.mlog = {L.DEBUG,'DMagician','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end
            
            self.CreateModel(); % Creating the dobot magician D&H parameter model

            % Orientating the dobot magician within the workspace
            self.model.base = self.model.base.T * baseTr; % Updating the base of the robot to input base tr
            self.homeQ = self.defaultRealQ; % Setting initial starting pose of robot
            self.PlotAndColourRobot(); % Plotting the dobot magician and associated ply models

            % Logging the creation of the Dobot Magician
            L.mlog = {L.DEBUG,'DMagician','Dobot Magician object created within the workspace'};
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the Dobot Magician model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % URL: https://en.dobot.cn/products/education/magician.html
            link(1) = Link([0    0.103+0.0362   0       -pi/2   0    0]);
            link(2) = Link([0    0              0.135    0      0   -pi/2]);
            link(3) = Link([0    0              0.147    0      0    0]);
            link(4) = Link([0    0              0.060    pi/2   0   -pi/2]);
            link(5) = Link([0   -0.05           0        0      0    pi]);
            
            % Qlims for each joint
            link(1).qlim = [-135 135]*pi/180;
            link(2).qlim = [5    80]*pi/180;
            link(3).qlim = [-5   85]*pi/180;
            link(4).qlim = [-180 180]*pi/180;
            link(5).qlim = [-85  85]*pi/180;

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end   

        %% Moving the DMagician to a Desired Transform
        function qMatrix = GetCartesianMovement(self, coordinateTransform)
            deltaT = self.movementTime/self.movementSteps; % Calculating discrete time step

            % Allocating memory to data arrays
            manipulability = zeros(self.movementSteps, 1);      % Array of measure of manipulability
            qMatrix = zeros(self.movementSteps, self.model.n);  % Array of joint angle states
            qdot = zeros(self.movementSteps, self.model.n);     % Array of joint velocities
            theta = zeros(3, self.movementSteps);               % Array of end-effector angles
            trajectory = zeros(3, self.movementSteps);          % Array of x-y-z trajectory

            % Getting the initial and final x-y-z coordinates
            initialTr = self.model.fkine(self.model.getpos()).T; % Getting the transform for the robot's current pose
            x1 = initialTr(1:3,4); % Extracting the x-y-z coordinates from the current pose transform
            x2 = coordinateTransform(1:3,4); % Extracting  the x-y-z coordiantes from the final pose transform

            % Getting the initial and final roll-pitch-yaw angles
            rpy1 = tr2rpy(initialTr(1:3,1:3)); % Getting the inial roll-pitch-yaw angles
            rpy2 = tr2rpy(coordinateTransform(1:3,1:3)); % Getting the final roll-pitch-yaw angles

            % Creating the movement trajectory
            s = lspb(0,1,self.movementSteps); % Create interpolation scalar
            for i = 1:self.movementSteps
                trajectory(:,i) = x1*(1-s(i)) + s(i)*x2; % Creating the translation trajectory
                theta(:,i) = rpy1*(1-s(i)) + s(i)*rpy2; % Creating the rotation trajectory
            end

            % Creating the transform for the first instance of the trajectory
            firstTr = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) trajectory(:,1); zeros(1,3) 1];
            q0 = self.model.getpos(); % Getting the initial guess for the joint angles
            qMatrix(1,:) = self.model.ikcon(firstTr, q0); % Solving the qMatrix for the first waypointx

            % Tracking the movement trajectory with RMRC
            for i = 1:self.movementSteps-1
                currentTr = self.model.fkine(qMatrix(i,:)).T; % Getting the forward transform at current joint states
                deltaX = trajectory(:,i+1) - currentTr(1:3,4); % Getting the position error from the next waypoint
                Rd = rpy2r(theta(1,i+1), theta(2,i+1), theta(3,i+1)); % Getting the next RPY angles (converted to rotation matrix)
                Ra = currentTr(1:3,1:3); % Getting the current end-effector rotation matrix
                
                Rdot = (1/deltaT) * (Rd-Ra); % Calcualting the roll-pitch-yaw angular velocity rotation matrix
                S = Rdot * Ra;
                linearVelocity = (1/deltaT) * deltaX; % Calculating the linear velocities in x-y-z
                angularVelocity = [S(3,2);S(1,3);S(2,1)]; % Calcualting roll-pitch-yaw angular velocity
                xdot = self.movementWeight*[linearVelocity; angularVelocity]; % Calculate end-effector matrix to reach next waypoint

                J = self.model.jacob0(qMatrix(i,:)); % Calculating the jacobian of the current joint state

                % Implementing Damped Least Squares
                manipulability(i,1) = sqrt(det(J*J')); % Calcualting the manipulabilty of the aubo i5
                if manipulability(i,1) < self.epsilon % Checking if manipulability is within threshold
                    lambda = (1 - (manipulability(i,1)/self.epsilon)^2) * self.maxLambda; % Damping Coefficient
                    
                else % If DLS isn't required
                    lambda = 0; % Damping Coefficient
                end

                invJ = inv(J'*J + lambda * eye(self.model.n))*J'; %#ok<MINV> % DLS inverse
                qdot(i,:) = (invJ * xdot)'; % Solving the RMRC equation
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot(i,:); % Updating next joint state based on joint velocities
            end
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