classdef Cyton < handle
    properties
        model;
        workspace = [-1 1 -1 1 -1 1];     
    end
    
    methods
        function self = Cyton()
        self.GetCytonRobot();
        end

%% GetCytonRobot
        function GetCytonRobot(self)
        pause(0.001);
        name = ['Cyton_',datestr(now,'yyyymmddTHHMMSSFFF')];

        L1 = Link('d',(0.054+0.066),'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-150),deg2rad(150)]);
        L2 = Link('d',0,'a',0,'alpha',-pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
        L3 = Link('d',0.13,'a',0,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-150),deg2rad(150)]);
        L4 = Link('d',0,'a',0.065,'alpha',pi/2,'offset', pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
        L5 = Link('d',0,'a',0.071,'alpha',pi/2,'offset', 0,'qlim',[deg2rad(-105),deg2rad(105)]);
        L6 = Link('d',0,'a',0,'alpha',-pi/2,'offset', -pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
        L7 = Link('d',0.129,'a',0,'alpha',0,'offset', pi,'qlim',[deg2rad(-150),deg2rad(150)]);

        self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
        end
%% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['cyton/cytonLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
%             if isempty(findobj(get(gca,'Children'),'Type','Light'))
%                 camlight
%             end  
            self.model.delay = 0;
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end