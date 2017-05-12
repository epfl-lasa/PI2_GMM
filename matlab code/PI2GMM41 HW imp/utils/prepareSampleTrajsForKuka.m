function [trajs] = prepareSampleTrajsForKuka(allTrajs, indexes, offset)
    n_steps_tot = size(allTrajs,3);
    for i = 1:length(indexes)
        temp = squeeze(allTrajs(indexes(i),:,:));
        
        qw = cos(offset(4)/2)*cos(temp(3,:)+offset(5)/2)*cos(offset(6)/2) + sin(offset(4)/2)*sin(temp(3,:)+offset(5)/2)*sin(offset(6)/2);
        qx = -cos(offset(4)/2)*sin(temp(3,:)+offset(5)/2)*sin(offset(6)/2) + sin(offset(4)/2)*cos(temp(3,:)+offset(5)/2)*cos(offset(6)/2);
        qy = cos(offset(4)/2)*sin(temp(3,:)+offset(5)/2)*cos(offset(6)/2) + sin(offset(4)/2)*cos(temp(3,:)+offset(5)/2)*sin(offset(6)/2);
        qz = cos(offset(4)/2)*cos(temp(3,:)+offset(5)/2)*sin(offset(6)/2) - sin(offset(4)/2)*sin(temp(3,:)+offset(5)/2)*cos(offset(6)/2);
        traj = [ ones(n_steps_tot,1).*offset(1) , temp(1,:)'+ offset(2) , temp(2,:)' + offset(3), qx', qy' ,qz',qw'];  % [px py pz qx qy qz qw], the simulation works in the XZ plane, axis [0 1 0] angle representation is transformed to quaternion

        str = sprintf('traj_%d',indexes(i));
        save(str, 'traj','-ascii');
        trajs{indexes(i)} = traj;
        figure
        subplot(2,1,1)
        plot(temp(1,:),temp(2,:),'b.');
        str = sprintf('trajectory %d',indexes(i));
        title(str)
        xlabel('x')
        ylabel('z')
        subplot(2,1,2)
        plot(temp(1,:),temp(3,:),'b.');
        str = sprintf('angle %d',indexes(i));
        title(str)
        xlabel('x')
        ylabel('angle y')
    end
end