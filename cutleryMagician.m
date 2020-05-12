clc
clf
clear all

%% Create the Workspace %%

%use background or environment functions to make workspace

%% Model DoBot
robot = Dobot;              %Calling the Dobot class
robot.CreateDobot();        %Calling the createDobot function from Dobot class to make model
q=zeros(1,5);
robot.base;
% base = transl(0.22,0.22,0);
% robot.model.base = base;
%robot.model.plotop = {'nojoints', 'noname', 'noshadow','nowrist','workspace',robot.workspace};
robot.model.plot(q,'scale', 1, 'workspace', robot.workspace);     %Plot model


% use switch case for determining movement of robot/end effector to box
% depending on which cutlery was picked up

%% Sensor Data???