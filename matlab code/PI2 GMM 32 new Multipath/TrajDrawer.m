%%% This class is a tool for drawing trajectory using a mouse on a Matlab
%%% figure. It samples the position of the mouse using a timer triggered by
%%% the callback function handles that are part of the properties of a
%%% figure object. A right click deleats the latest demo.

classdef TrajDrawer < handle
    properties
        figHandle;
        axesHandle;
        timerHandle;
        dataPoints = cell(1,1);
        currentStep = 1;
        demoNb = 0;
    end
    
    methods
        function obj = TrajDrawer()
            obj.figHandle = figure;
            %obj.axesHandle = axes('Xlim',[-2 ,15],'Ylim', [-10 ,10]);
            %obj.axesHandle = axes('Xlim',[-2 ,12],'Ylim', [-2 ,12]);
            obj.axesHandle = axes('Xlim',[-10 ,10],'Ylim', [-10 ,10]);
            obj.drawWorld();
            obj.timerHandle = timer('TimerFcn',@obj.sampleFcn,'ExecutionMode','fixedRate','Period',0.1);
            set(obj.figHandle,'WindowButtonUpFcn',@obj.stopSamplingFnc);
            set(obj.figHandle,'WindowButtonDownFcn',@obj.startSamplingFnc);
            set(obj.figHandle,'WindowButtonMotionFcn',@obj.moveFnc);
            'constr'
            
        end
        function startSamplingFnc(obj,src,evt)
            button = get(src, 'selectionType');
            if strcmp(button, 'normal')
                obj.demoNb = obj.demoNb +1
                start(obj.timerHandle);
                'hello'
            elseif strcmp(button, 'alt')
                obj.dataPoints = obj.dataPoints(1:end-1);
                obj.demoNb = obj.demoNb -1
            end
        end

        function stopSamplingFnc(obj,src,evt)
            button = get(src, 'selectionType');
            if strcmp(button, 'normal')
                stop(obj.timerHandle);
            end
            obj.drawWorld();
            hold on
            for cnt1 = 1:obj.demoNb
                scatter(obj.dataPoints{cnt1}(1,:), obj.dataPoints{cnt1}(2,:),'.')
            end
            hold off
            obj.currentStep = 1;
            'goodbye'
        end

         function moveFnc(obj,src,evt)
    
         end
        
        function sampleFcn(obj,src,evt)
       
            currentPoint = get(obj.axesHandle, 'CurrentPoint');
            currentPointProj = currentPoint(1,1:2)';    % assuming top view
            obj.dataPoints{obj.demoNb}(:,obj.currentStep) = currentPointProj;
            obj.currentStep = obj.currentStep + 1;

        end
        
        function drawWorld(obj)
            clf
            %obj.axesHandle = axes('Xlim',[-2 ,15],'Ylim', [-10 ,10]);
            % obj.axesHandle = axes('Xlim',[-2 ,12],'Ylim', [-2 ,12]);
            obj.axesHandle = axes('Xlim',[-10 ,10],'Ylim', [-10 ,10]);
            hold on
%             line([-2, -2], [-2, 2]);
%             line([-2, -2], [3, 6]);
%             line([-2, -2], [-3, -6]);
%             scatter(0,0,'x')
%             scatter(0,0,'o')

               
%              scatter(8,8,'o')
%              scatter(8,8,'x')
%              scatter(12,6,'o')
%              scatter(12,6,'x')
%              scatter(0,0,'o')
%              scatter(0,0,'x')

%              scatter(0,0,'o')
%              scatter(0,0,'x')
%              
%             scatter(-3,0,'o')
%             scatter(-5,3,'o')
%             scatter(5,3,'o')
%             scatter(-8,0,'o')


%              line([-2, -2], [-2, 2]);
%              line([-2, 2], [-2, -2]);
%              line([2, 2], [-2, 2]);
%              scatter(0,0,'o')
%              scatter(-6,-2,'o')

                scatter(0,0,'o')
                scatter(-8,0,'x')
                scatter(8,0,'x')
              %  scatter([-6 -4 -2 0 2 4 6],[2 0 -2 0 -3 -3 0], 80, 'gx')
            
               scatter([-6 -4 -2 0 2 3 5 6],[-3 0 3 0 0 -4 -4 0], 80, 'gx')
             
                
            hold off
        end
    end
end