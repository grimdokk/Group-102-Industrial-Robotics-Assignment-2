%% Hand
classdef Hand < RobotBaseClass
    %% Robot Class Properties
    % Code has been integrated from UTS' robotCow code functionality
    % https://canvas.uts.edu.au/courses/27375/pages/subject-resources?module_item_id=1290469

    % Constant Properties
    properties (Access = public, Constant)
        handCount = 2; % Default number of hands to create/plot
        WORKSPACE_DIMENSIONS = [-3 3 -3 3 -0.01 -0.01]; % Dimension of workspace
    end

    % Non-constant properties
    properties (Access = public)
        handModels; % Cell structure to store the hands created
        plyFileNameStem = 'Hand'; % Default name for hand
    end

    %% ...structors
    methods
        %% Constructor for hand objects
        function self = Hand(baseTr,L)
            % Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'Hand','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end

            % Plotting the hands within the workspace
            for i = 1:self.handCount
                % Creating the hand D&H link model
                self.handModels{i} = self.CreateModel(['hand',num2str(i)]);
                self.handModels{i}.base = baseTr; % Updating base pose of hands
                % self.handModels{1, 1}.points{1, 2} = self.handModels{1, 1}.points{1, 2} + baseTr(1:3,4)';

                % Plotting the hands
                plot3d(self.handModels{i},pi,'workspace',self.WORKSPACE_DIMENSIONS,'view', ...
                    [-30,30],'delay',0,'noarrow','nowrist', 'notiles');

                % Logging creation of hands
                L.mlog = {L.DEBUG,'Hand',['Hand ',num2str(i),' created within the workspace']};
            end
        end
    end

    methods (Static)
        %% Creating the hand link models
        function model = CreateModel(name)
            % Setting default name scheme for hand if no argin
            if nargin < 1
                name = 'hand';
            end

            % Reading the hand model ply file and creating links
            [faceData,vertexData] = plyread('Hand.ply','tri');
            link(1) = Link([0   0.01   0   pi   0   0]);
            model = SerialLink(link,'name',name); % Creating link model

            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end

    end
end
