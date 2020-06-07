classdef Jaco < handle
    properties
        model;
        workspace = [-2 2 -2 2 -0.5 1];
        scale = 0.1;
        toolModelFileName = []; %For importing model of robot if files available 
    end
    
    methods
        %self = creating a function for the rest of the methods to refer
        %to: ie self = Dobot class
        
         function self = Jaco(toolModelAndTCPFilenames)
              if nargin > 0
                 self.toolModelFilename = toolModelAndTCPFilenames{1};
                 
              end
         end
             
    %These DH Parameters are from the Kinova Robotics User Guide
        function GetJacoRobot(self)

        pause(0.001);

        L1 = Link('d', 0.2755, 'a', 0, 'alpha', pi/2, 'offset', 0,'qlim', [deg2rad(-360) deg2rad(360)]);
        L2 = Link('d', 0, 'a', 0.4100, 'alpha', pi, 'offset',  -pi/2, 'qlim', [deg2rad(47) deg2rad(313)]);
        L3 = Link('d', -0.0098, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(19) deg2rad(341)]);
        L4 = Link('d', -(0.2073+0.0741), 'a', 0, 'alpha', pi/2, 'offset', 0,'qlim', [deg2rad(-360) deg2rad(360)]);
        L5 = Link('d', -0.0743, 'a', 0, 'alpha', pi/2, 'offset', pi, 'qlim', [deg2rad(65) deg2rad(295)]);
        L6 = Link('d', -(0.0741+0.1600), 'a', 0, 'alpha', pi, 'offset', 0,'qlim', [deg2rad(-360) deg2rad(360)]);

        self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'Jaco');

        end
    
    %%PlotandColour
    
        function PlotAndColourRobot(self)

        for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['JacoLink',num2str(linkIndex),'.ply'],'tri'); 
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
        end

        %Display

        self.model.plot3d(zeros(1, self.model.n), 'noarrow', 'workspace', self.workspace);
        self.model.delay = 0;

        %Colour model if colour data in ply file
        for linkIndex = 0:self.model.n
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
            try
                h.link(linkIndex + 1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                , plyData{linkIndex+1}.vertex.green ...
                                                                , plyData{linkIndex+1}.vertex.blue]/255 
                h.link(linkIndex + 1).Children.FaceColor = 'interp';
            catch ME_1
                disp(ME_1);
                continue;
            end
        end
        end
  end
       
end

    