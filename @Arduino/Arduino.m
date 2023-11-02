%% Arduino Controlling Emergency Stop Button
classdef Arduino < handle
    % Constant Properties
    properties
        port = ""; % Provide the port number that the arduino is connected to
        board = "Uno"; % Type of Arduino Board
        pin =  "D13"; % Pin number where the arduino get reads
    end
    % Non-Constant Properties
    properties
        arduinoSeriPort; % Variable for serial port connection
        state = 0; % Emergency stop state
    end
%% Methods
    methods
        %% Object creation for Arduino
        function self = Arduino()
            % Create serial port connection to arduino board
            self.arduinoSeriPort = serialport(self.port, 9600);
        end
        %% Check the state of the Emergency Stop button
        function state = checkButton(self)
            % Send data to ask for the state of the Emergency Stop button
            % from arduino
            write(self.arduinoSeriPort, 1, "uint8");
            % Read return data
            data = readline(self.arduinoSeriPort);
            % Convert and store the state of Emergency Stop button
            state = str2double(data);
        end
        %% Force the Emergency Stop button to be ON state
        function state = emgON(self)
            % Change the state to be ON to arduino
            write(self.arduinoSeriPort, 2, "uint8");
            % Read the given data
            data = readline(self.arduinoSeriPort);
            % Convert and store tje state of Emergency Stop button
            state = str2double(data);
        end
        %% Force the Emergency Stop button to be  state
        function state = emgOFF(self)
            % Change the state to be OFF to arduino
            write(self.arduinoSeriPort, 1, "uint8");
            % Read the given data
            data = readline(self.arduinoSeriPort);
            % Convert and store tje state of Emergency Stop button
            state = str2double(data);
        end
    end
end