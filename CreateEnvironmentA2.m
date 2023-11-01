 classdef CreateEnvironmentA2 
    properties
        figureHandle
        axisHandle
        robot
    end
    
    methods
        % Constructor
        function obj = CreateEnvironmentA2()
            clc
           % Create a figure and axes for visualization
            obj.figureHandle = figure;
            obj.axisHandle = axes('Parent', obj.figureHandle, 'XLim', [-5, 5], 'YLim', [-5, 5], 'ZLim', [0, 5]);
            % Log a message to indicate the start of environment setup
            disp('Initializing the environment...');
            obj.initializeWorkspace();
            obj.insertConcreteFloor(obj.axisHandle);
            
            % Import and place objects in the environment
            obj.importTable();
            obj.importHuman();
            obj.importEmergencyStop();
            obj.importFireExtinguisher();
            % obj.importFence();
            % obj.importAndPlaceBricks();
            % Create the robot and store it as a property
            robotBase = transl(0.5, 0, 0.5);
            q = zeros(1, 7);
            obj.robot = LinearUR3(robotBase);
            % Log a message to indicate the completion of environment setup
            disp('Environment setup completed.');

        end
        
        function initializeWorkspace(obj)
            % Initialise workspace by setting axes
            hold(obj.axisHandle, 'on');
        end
        

    end
    
    methods (Static)
        function insertConcreteFloor(axisHandle)
            % Insert concrete floor
            surf(axisHandle, [-5, -5; 5, 5], [-5, 5; -5, 5], [0.01, 0.01; 0.01, 0.01], ...
                'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
            hold(axisHandle, 'on');
        end
        
        function importTable()
            % Import table
            [f, v, data] = plyread('tableBrown2.1x1.4x0.5m.ply', 'tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            trisurf(f, v(:, 1), v(:, 2), v(:, 3), ...
                'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
        end

        function importHuman()
            % Import Human
            person = PlaceObject('personMaleCasual.ply', [-4, 0, 0]);
            verts = [get(person, 'Vertices'), ones(size(get(person, 'Vertices'), 1), 1)] * trotz(-pi);
            set(person, 'Vertices', verts(:, 1:3));
        end
        
        function importEmergencyStop()
            % Import Fire extinguisher
            e_stop = PlaceObject('emergencyStopWallMounted.ply', [1, -4.99, 1.5]);
            verts = [get(e_stop, 'Vertices'), ones(size(get(e_stop, 'Vertices'), 1), 1)];
            set(e_stop, 'Vertices', verts(:, 1:3));
        end

        function importFireExtinguisher()
            % Import Fire extinguisher
            fire_e = PlaceObject('fireExtinguisherElevated.ply', [-4.75, 0, 0.25]);
            verts = [get(fire_e, 'Vertices'), ones(size(get(fire_e, 'Vertices'), 1), 1)] * trotz(-pi/2);
            set(fire_e, 'Vertices', verts(:, 1:3));
        end
        
        % function importFence()
        %     % Import fence
        %     fence = PlaceObject('fenceAssemblyGreenRectangle4x8x2.5m.ply', [0, 1, -0.99]);
        %     verts = [get(fence, 'Vertices'), ones(size(get(fence, 'Vertices'), 1), 1)];
        %     verts(:, 1) = verts(:, 1) * 0.75;
        %     scaled_fence = imresize(fence, 0.6);
        %     set(scaled_fence, 'Vertices', verts(:, 1:3));
        % end
    end
end
