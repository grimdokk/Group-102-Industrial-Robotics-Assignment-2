%% AUBO i5 Robot
% https://www.aubo-cobot.com/public/i5product3?CPID=i5

classdef AUBOi5 < RobotBaseClass
    %% Robot Class Properties
    % Constant Properties
    properties (Access = public, Constant)
        % The initial or default pose for the Aubo i5
        iniJointAng = deg2rad([0 135 -105 150 -90 0]); 
        % The count of steps assigned to movement trajectories
        mvmSteps = 100; 
        % The duration of movements executed by the Aubo i5
        mvmTime = 10;
        % The maximum level of manipulability that would necessitate the application of Damped Least Squares
        maxEpsilon = 0.01; 
        % Matrix used to weight the velocity vector for movement
        mvmWeight = diag([1 1 1 1 1 1]); 
        % Parameter value employed in Damped Least Squares
        maxLam = 5E-2; 
        % Slight modification in the trajectory for a small angle change
        delt = 2*pi/1000; 
    end

    % Non-constant Properties
    properties (Access = public)
        % The present joint angles of the Aubo i5
        crrntJointAng; 
        % A variable to store the tool (2F-85 Gripper) utilized by the Aubo i5
        tool = cell(1,2); 
        % The stem name employed to locate corresponding ply files
        plyFileNameStem = 'AUBOi5'; 
        % Holds the ellipsoid robot object for collision checking
        ellipsis; 
        % Data structures that contain collision ellipsoid information
        ellipsoids = cell(1,7); 
        % A structure that defines the positions of link centers used for ellipsoid updates
        linkCent = zeros(6,3); 
        % A structure that specifies the radii of ellipsoids associated with links
        linkRadii = zeros(6,3); 
        ctrOffset =  [0 0 -0.25;
                        -0.3 0 -0.15;
                         -0.3 0 -0.05;
                               0 0.1 0;
                               0 -0.1 0;
                               0 0 0];
    end

    %% ...structors
    methods
        %% Constructor for Aubo i5 Robot
        function self = AUBOi5(baseTr,L)
            % Configuring the default base transform if it has not been specified within the constructor inputs
            if nargin < 2
                % Generating a log file if it has not been provided, indicating that it has not been created yet
                L = log4matlab('LogFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Recording that the default base transform has been employed
                    L.mlog = {L.DEBUG,'AuboI5','The base transform is not configured. The default base transform is applied.'};
                    % Setting default base transform
                    baseTr = eye(4); 
                end
            end
            % Developing the D&H parameter model for the Aubo i5
            self.crtModel(); 
            % Positioning the Aubo i5 within the workspace
            self.model.base = self.model.base.T * baseTr; 
            self.ellipsis.base = self.ellipsis.base.T * baseTr;
            % Configuring the initial pose of the Aubo i5
            self.homeQ = self.iniJointAng; 

            % Visualizing the Aubo i5 and its associated PLY models
            self.PlotAndColourRobot();

            % Recording the creation of the Aubo i5
            L.mlog = {L.DEBUG,'AuboI5','An instance of the AUBO - i5 has been instantiated within the workspace'};

            % Establishing the 2F-85 gripper and affixing it to the end-effector of the Aubo i5
                % Modifying the end-effector transform property.
            self.updtToolTr; 
            for gripperFinger = 1:2
                self.tool{1,gripperFinger} = TwoFingeredGripper(self.toolTr, gripperFinger, L);
            end
        end

        %% Creation of a serial link using D&H parameters
        function crtModel(self)
            % D&H parameters for the Aubo i5 model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % https://www.aubo-cobot.com/public/i5product3?CPID=i5
            link(1) = Link([0    0.1215   0        pi/2   0   0]);
            link(2) = Link([0    0        0.4080   0      0   0]);
            link(3) = Link([0    0        0.3760   0      0   0]);
            link(4) = Link([0   -0.1215   0        pi/2   0   0]);
            link(5) = Link([0    0.1025   0        pi/2   0   0]);
            link(6) = Link([0    0.0940   0        0      0   0]);

            % Qlims for each joint
            % https://www.aubo-cobot.com/public/i5product3?CPID=i5
            link(1).qlim = [-90   90]*pi/180;
            link(2).qlim = [5     175]*pi/180;
            link(3).qlim = [-175  175]*pi/180;
            link(4).qlim = [-360  360]*pi/180;
            link(5).qlim = [-175  175]*pi/180;
            link(6).qlim = [-360  360]*pi/180;

            % Generating the serial link object.
            self.model = SerialLink(link,'name',self.name);
            self.ellipsis = SerialLink(link,'name',[self.name,'_ellipsis']);
        end

        %% Updating End Effector Transform
        function updtToolTr(self)
            % Updating the robot's tool transformation (toolTr) property
            % This is utilized to adjust the gripper's pose to match the end-effector
                % Retrieving the current joint angles of the Aubo i5
            self.crrntJointAng = self.model.getpos(); 
                % Modifying the toolTr property
            self.toolTr = self.model.fkine(self.crrntJointAng).T; 
        end

        %% Retrieving the qMatrix to restore the Aubo i5 to its original joint states
        function qMatrix = rtnAUBOiniPos(self)
            % Obtaining the current joint states of the robot
            crrntJointSt = self.model.getpos();
            
            % Generating a qMatrix using the "jtraj" function
            qMatrix = jtraj(crrntJointSt, self.iniJointAng, self.mvmSteps);
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
            x1 = iniTr(1:3,4); 
               % Obtaining the x-y-z coordinates from the final pose transformation
            x2 = coorTransf(1:3,4); 

            % Obtaining the initial and final roll-pitch-yaw angles
                % Retrieving the initial roll-pitch-yaw angles
            rpy1 = tr2rpy(iniTr(1:3,1:3)); 
                % Retrieving the final roll-pitch-yaw angles
            rpy2 = tr2rpy(coorTransf(1:3,1:3)); 

            % Generating the movement trajectory
                % Generate an interpolation scalar
            s = lspb(0,1,self.mvmSteps); 
            for i = 1:self.mvmSteps
                    % Generating the translation trajectory
                traj(:,i) = x1*(1-s(i)) + s(i)*x2; 
                    % Generating the rotation trajectory
                theta(:,i) = rpy1*(1-s(i)) + s(i)*rpy2; 
            end

            % Establishing the transformation for the initial instance of the trajectory
            firstTr = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) traj(:,1); zeros(1,3) 1];
                % Obtaining the initial estimate for the joint angles
            q0 = self.model.getpos(); 
                % Solving the qMatrix for the initial waypoint
            qMatrix(1,:) = self.model.ikcon(firstTr, q0); 

            % Following the movement trajectory using RMRC
            for i = 1:self.mvmSteps-1
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
                    % Calculating the manipulability of the Aubo i5
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
                % Solving the RMRC equation
                qdot(i,:) = (invJ * xdot)'; 
                % Adjusting the next joint state based on the joint velocities
                qMatrix(i+1,:) = qMatrix(i,:) + deltT * qdot(i,:); 
            end
        end     
        
        %% Generating an ellipsoid around each robot link
        function UpdateEllipsis(self, q)
            % Generating an array of 4x4 transformation matrices associated with robot links
            linkTransf = zeros(4,4,(self.ellipsis.n)+1); 
                % Defining the first transformation as the base transform
            linkTransf(:,:,1) = self.ellipsis.base; 

            piFlag = 0;
            
            mult =    [0.5 0.5 0.5;
                      0.66 0.66 0.66;
                      0.66 0.66 0.66;
                      0.4 0.6 0.4;
                      0.4 0.4 0.4;
                      0.5 0.25 0.8];
            
            % Getting the link data of the robot links
            links = self.ellipsis.links;

            for i = 1:length(links)
                L = links(1,i);
                
                crrntTransf = linkTransf(:,:, i);
                
                crrntTransf = crrntTransf * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                linkTransf(:,:,i + 1) = crrntTransf;

                centTr = (crrntTransf-linkTransf(:,:,i))/2;

                self.linkCent(i,1) = self.ctrOffset(i,1) + centTr(3,1);
                self.linkCent(i,2) = self.ctrOffset(i,2) + centTr(3,2);
                self.linkCent(i,3) = self.ctrOffset(i,3) + centTr(3,3);
            end

            self.updtToolTr;
            self.linkCent(6,1) = self.toolTr(1,4);
            self.linkCent(6,2) = self.toolTr(2,4);
            self.linkCent(6,3) = self.toolTr(3,4);

            for i = 1:length(links)

                A = 0.001;
                D = 0.001;

                if(i >= 2)

                    if(links(i-1).alpha == pi/2)
                        piFlag = ~piFlag;
                    end

                    if(piFlag)
                        A = A + links(i).d;
                        D = D + links(i).a;
                    else
                        A = A + links(i).a;
                        D = D + links(i).d;
                    end
                else
                     A = A + links(i).a;
                     D = D + links(i).d;
                end

                if (D > 0.001)
                    radii = [D, 0.2, 0.2];
                else
                     radii = [A, 0.2, 0.2];
                end

                self.linkRadii(i,1) = radii(1);
                self.linkRadii(i,2) = radii(2);
                self.linkRadii(i,3) = radii(3);

                [X, Y, Z] = ellipsoid(self.ctrOffset(i,1), self.ctrOffset(i,2), self.ctrOffset(i,3), radii(1), radii(2), radii(3));

                self.ellipsis.points{i+1} = [X(:)*mult(i,1),Y(:)*mult(i,2),Z(:)*mult(i,3)];
                self.ellipsis.faces{i+1} = delaunay(self.ellipsis.points{i+1}); 

            end

            radii = [0.1,0.1,0.1];

            [X, Y, Z] = ellipsoid(0, 0, 0, radii(1), radii(2), radii(3));

            self.ellipsis.points{1} = [X(:),Y(:),Z(:)];
            self.ellipsis.faces{1} = delaunay(self.ellipsis.points{1}); 

            self.ellipsis.plot3d(q);
        end

        %% Verifying if a collision is taking place with a model
        function collStat = checkCollisions(self, jointAngles, model)
            % Establishing the default return condition to indicate no current collisions.
            collStat = false; 
            
            % Constructing an array of 4x4 transformation matrices related to robot links.
            linkTransf = zeros(4,4,(self.ellipsis.n)+1);     
                % Assigning the first transformation as the base transform.
            linkTransf(:,:,1) = self.ellipsis.base; 

            % Retrieving the data associated with the robot links.
            linkData = self.ellipsis.links;

            % Obtaining the points on the model that require inspection.
            points = [model.points{1,2}(:,1), model.points{1,2}(:,2), model.points{1,2}(:,3)];
            points = points(1:3) + model.base.t(1:3)';

            % A for loop to obtain the remaining link transformations.
            for i = 1:self.ellipsis.n
                % Computing the link transformations using the provided link data.
                linkTransf(:,:,i+1) = linkTransf(:,:,i) * trotz(jointAngles(i)) * transl(0,0,linkData(i).d) ...
                * transl(linkData(i).a,0,0) * trotx(linkData(i).alpha);
            end

            % Iterating through each ellipsoid to inspect for potential collisions.
            for i = 1:size(linkTransf, 3)
                % Modifying the points of the model in relation to the links on the robot.
                mdlPointtoLink = (inv(linkTransf(:,:,i)) * [points, ones(size(points,1),1)]')'; %#ok<MINV>
                % Obtaining the relative x-y-z coordinates.
                updtModelPoints = mdlPointtoLink(:,1:3); 
    
                % Assessing the algebraic distance of these points.
                algeDist = self.getAlgeDist(updtModelPoints, self.linkCent(i,:), self.linkRadii);
                    
                % Verifying if the model is within the light curtain, which means there is an algebraic distance of less than 1 with any of the points mentioned above.
                if(find(algeDist < 1) > 0)
                    % An object has been detected within the light curtain.
                    collStat = true; 
                    % Returning immediately upon detecting an object within the light curtain.
                    return; 
                end
            end
        end

        function points = objectCollision(self, ellipsisModel)
            % For each ellipsoid (link), check whether the algebraic distance of points is less than 1.

            points = [];
            
            % Storing every 50 points in the points array.
            for l = 1:width(ellipsisModel.points())
                ellipsisPoints = ellipsisModel.points{l};
                for p = 1:height(ellipsisModel.points{l})
                    point = ellipsisPoints(p,:);
                    points = [points; point];
                end
            end

            for l = 1:self.model.n

                    %dis = GetAlgebraicDist(points, self.linkCentres(l,:), self.linkRadii(l,:));

            end 
        end


        function prisms(self, q)

            linkTransforms = zeros(4,4,(self.ellipsis.n)+1);                
            linkTransforms(:,:,1) = self.ellipsis.base;

            links = self.ellipsis.links;

            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = linkTransforms(:,:, i);
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                linkTransforms(:,:,i + 1) = current_transform;

                centreTr = (current_transform-linkTransforms(:,:,i))/2;

                self.linkCent(i,1) = self.ctrOffset(i,1) + centreTr(3,1);
                self.linkCent(i,2) = self.ctrOffset(i,2) + centreTr(3,2);
                self.linkCent(i,3) = self.ctrOffset(i,3) + centreTr(3,3);
            end

            for i = 1:length(links)

                d1 = 0;
                d2 = 0;
                d3 = 0;
                x1 = linkTransforms(1,4,i);
                x2 = linkTransforms(1,4,i);
                y1 = linkTransforms(2,4,i);
                y2 = linkTransforms(2,4,i);
                z1 = linkTransforms(3,4,i);
                z2 = linkTransforms(3,4,i);
                for k = x1:0.05:x2
                    d1 = k;
                    for j = y1:0.05:y2
                        d2 = j;
                        for i = z1:0.05:z2
                            d3 = i;
                            self.ellipsis.points{i}
                        end
                    end
                end 

                % self.ellipsis.points{i+1} = [X(:)*mult(i,1),Y(:)*mult(i,2),Z(:)*mult(i,3)];
                % self.ellipsis.faces{i+1} = delaunay(self.ellipsis.points{i+1}); 

            end


        end


    end

    %% Static Methods
    methods (Static)
        %% Getter for the Algerbraic Distance between Objects and Light Curtain
        function algeDist = getAlgeDist(points, centPoint, radii)
            algeDist = ((points(:,1)-centPoint(1))/radii(1)).^2 ...
                  + ((points(:,2)-centPoint(2))/radii(2)).^2 ...
                  + ((points(:,3)-centPoint(3))/radii(3)).^2;
        end

    end
end
