% Assignment 2: Cutlery Magician ------------------------------------------------------

%Modelling of DoBot in Matlab using suggested limits

classdef Dobot < handle
    properties
        model;
        workspace = [-0.4 1, -0.4 1, -0.4 1];
        base = [0,0,0];
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';        
       
    end
    
    methods
        %self = creating a function for the rest of the methods to refer
        %to: ie self = Dobot class
        
         function self = Dobot(toolModelAndTCPFilenames)
              if nargin > 0
                 self.toolModelFilename = toolModelAndTCPFilenames{1};
                 
             end
             
         end
        
        %This function creates the serial links of the robot according to
        %DH parameters
        function CreateDobot(self)
           
            L1 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-pi/2 pi/2],'offset', 0); 
            L2 = Link('d',0,'a',0.134,'alpha',0,'qlim',[deg2rad([5 80])], 'offset', -pi/2); 
            L3 = Link('d',0,'a',0.148,'alpha',0,'qlim',[deg2rad([15 170])],'offset', 0); 
            L4 = Link('d',0,'a',0.043,'alpha',pi/2,'qlim',[-pi/2 pi/2],'offset', 0);
            L5 = Link('d',0,'a',0.061,'alpha',pi,'qlim',[deg2rad([-85 85])],'offset', 0);

            self.model = SerialLink([L1,L2,L3,L4,L5], 'name', 'DoBot');      %Create DoBot; call function in class.property = set
            
                               
        end
    end
end

