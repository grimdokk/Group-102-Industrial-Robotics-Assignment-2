clf; clear; clc;
model = 4;

figure(1);

if model == 1
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);
    
    % Creating 2F-85 gripper and attaching it to the Aubo i5 end-effector
    twoFingeredGripper = []; % Creating cell structure to store gripper fingers
    for i = 1:2
        twoFingeredGripper{i} = TwoFingeredGripper(eye(4),i,L);
    end
    
    twoFingeredGripper{1}.model.teach([0 45 45]*pi/180);
    twoFingeredGripper{2}.model.teach([0 45 45]*pi/180);
end

if model == 2
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    figure(1);
    hold on; axis(LabAssessment2.axisLimits); camlight;
    LabAssessment2.CreateEnvironment(L);

    % Creating aubo i5
    r = AuboI5(eye(4),L);
    axis([-1 1 -1 1 -1 1]);

    r.model.teach(r.model.getpos());
end

if model == 3
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);
    


    % Creating the GUI object
    guiWindow = GUI;
    guiWindow.LoadLogFile(L); % Loading the logfile into the gui class

end

if model == 4
    
       
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    figure(1); % Creating figure to simulate robots
    hold on; axis(LabAssessment2.axisLimits); camlight;

    LabAssessment2.CreateEnvironment(L);

    cards = PlayingCards(LabAssessment2.auboOrigin*transl(0.25,0.5,0.005),L);

    auboI5 = AuboI5(LabAssessment2.auboOrigin,L); % Spawning the Aubo i5 and associated 2F-85 gripper
    dobotMagician = DMagician(LabAssessment2.auboOrigin*transl(0,0.5,0)); % Spawning the Dobot Magician and associated suction gripper
    
    cards.cardModels{1}.base = auboI5.model.fkine(auboI5.model.getpos()).T * trotz(pi/2) * trotx(pi/2) * transl(0, 0.2, -0.01) ; %card to closed gripper mounting
    cards.cardModels{1}.base = dobotMagician.model.fkine(dobotMagician.model.getpos()).T * trotz(pi/2) * transl(0, 0, -0.045) ; % card to dobot mount 
    
    card_transform = eye(4);
    card_transform(1:3, 4) = [0.2, 0.5, 0.075];
    rotMatrix = eul2rotm([pi/2 0 0]);
    card_transform(1:3, 1:3) = rotMatrix; %transform for card on table 

    q_cardup = [0 15 45 120 -90]*pi/180; %q for raised dobot

    transform_cardpos = eye(4); 
    transform_cardpos(1:3, 4) = [0.2222, 0.32, 0.25];
    rotMatrix = eul2rotm([0 0 -pi/2])*eul2rotm([90 0 0]*pi/180);
    transform_cardpos(1:3, 1:3) = rotMatrix; %transform for aubo to get card from dobot

    transform = eye(4);
    transform(1:3, 4) = [0.5, 0, 0.2];
    rotMatrix1 = eul2rotm([-90 0 220]*pi/180);
    rotMatrix2 = eul2rotm([90 0 0]*pi/180);
    rotMatrix = rotMatrix1*rotMatrix2;
    transform(1:3, 1:3) = rotMatrix; %transform for aubo to place card on holder (other positions to be confirmed)

    
    
    
    cards.cardModels{1}.animate(0);
    auboI5.tool{1,2}.model.animate([0 92.5 4]*pi/180);
    auboI5.tool{1,1}.model.animate([0 92.5 4]*pi/180);
    drawnow;
    pause;
    
    gripperclosematrix = jtraj([0 45 45]*pi/180, [0 92.5 4]*pi/180, 100);
    gripperopenmatrix = jtraj([0 92.5 4], [0 45 45], 100);

    card_transform = eye(4);
    card_transform(1:3, 4) = [0.2, 0.5, 0.075];
    rotMatrix = eul2rotm([pi/2 0 0]);
    card_transform(1:3, 1:3) = rotMatrix;

    q_cardup = [0 15 45 120 -90]*pi/180;
    q_cardpos = dobotMagician.model.ikcon(card_transform);

    
    qmatrix_dobot_getcard = jtraj(q_cardup, q_cardpos, 200);
    qmatrix_dobot_putcard = jtraj(q_cardpos, q_cardup, 200);

    

    

    for i = 1:1:size(qmatrix_dobot_getcard,1)

        dobotMagician.model.animate(qmatrix_dobot_getcard(i,:));
        

        
        pause(0.01);
        drawnow;

    end

    for i = 1:1:size(qmatrix_dobot_putcard,1)

        dobotMagician.model.animate(qmatrix_dobot_putcard(i,:));

        cards.cardModels{1}.base = dobotMagician.model.fkine(dobotMagician.model.getpos()).T * trotz(pi/2) * transl(0, 0, -0.045) ;
        cards.cardModels{1}.animate(0);
        
        
        pause(0.01);
        drawnow;

    end

    cards.cardModels{1}.base

    
    transform = eye(4);
    transform(1:3, 4) = [0.5, 0, 0.2];
    rotMatrix1 = eul2rotm([-90 0 220]*pi/180);
    rotMatrix2 = eul2rotm([90 0 0]*pi/180);
    rotMatrix = rotMatrix1*rotMatrix2;
    transform(1:3, 1:3) = rotMatrix;
    disp(transform);

    % qmatrix = auboI5.GetQuaternionRMRC(transform);
    % currentTr = auboI5.model.fkine(auboI5.model.getpos()).T;
    
    qcurr_aubo = auboI5.model.getpos;
    % qnext_aubo = auboI5.model.ikcon(transform);
    


    transform_cardpos = eye(4); 
    transform_cardpos(1:3, 4) = [0.2222, 0.32, 0.25];

    rotMatrix = eul2rotm([0 0 -pi/2])*eul2rotm([90 0 0]*pi/180);
    transform_cardpos(1:3, 1:3) = rotMatrix;
    qnext_aubo = auboI5.model.ikcon(transform_cardpos);

    qmatrix = jtraj(qcurr_aubo, qnext_aubo, 100);
    qMatrixAubo = auboI5.GetCartesianMovementRMRC(transform_cardpos);
    




    for i = 1:1:size(qMatrixAubo,1)
        currentTr = auboI5.model.fkine(auboI5.model.getpos()).T;
        distToGoal = sqrt((currentTr(1,4)-transform(1,4))^2 + (currentTr(2,4)-transform(2,4))^2 + (currentTr(3,4)-transform(3,4))^2);
        disp(distToGoal);

        auboI5.model.animate(qMatrixAubo(i,:));
        auboI5.UpdateToolTr;

        for j = 1:2

            auboI5.tool{1,j}.UpdateGripperPosition(auboI5.toolTr, j);
            

        end
            % auboI5.tool{1,2}.UpdateGripperPosition(auboI5.toolTr);


        pause(0.01);

        drawnow;
    end

    

end 

if model == 5
L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    figure(1); % Creating figure to simulate robots
    hold on; axis(LabAssessment2.axisLimits); camlight;

    auboI5 = AuboI5(LabAssessment2.auboOrigin,L); % Spawning the Aubo i5 and associated 2F-85 gripper
    auboI5.UpdateEllipsis();
end

% pause;