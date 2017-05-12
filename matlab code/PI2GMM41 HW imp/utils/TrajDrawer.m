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
            obj.axesHandle = axes('Xlim',[-8 ,4],'Ylim', [-4 ,6]);
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
            obj.axesHandle = axes('Xlim',[-12 ,2],'Ylim', [-2 ,2]);
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

            nPoints = 20;
            for cnt1 = 1: nPoints
                forceFieldTraj(1,cnt1) = -10* (1 - cnt1/nPoints);
                forceFieldTraj(2,cnt1) = sin(2*pi*(1 - cnt1/nPoints));
            end
            scatter(forceFieldTraj(1,:),forceFieldTraj(2,:))
            
            hold off
        end
    end
end