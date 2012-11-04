function t = getT(inarr)
% This function will input an array with the first column containing time
% values, and each remaining column containing velocities, find the taus of
% all of the motors being tested.

    times = inarr(:,1);
    % Times are the time values
    velocities = inarr(:,2:end);
    % These are columns of velocities
    [a, numcol] = size(velocities);
    % Finds the total number of motors to be tested
    t = [];
    for col = 1:numcol
        % Finds t for each motor
        motor_vel = velocities(:,col);
        fvel = mean(motor_vel(end-10:end));
        % Finds the velocity that the data approaches
        dif = motor_vel - .632*fvel;
        [a, minpos] = min(abs(dif));
        % Finds the position whose velocity is closest to .632 of the final
        % velocity
        if length(minpos)>1
            minpos = minpos(1);
        end
        t = [t times(minpos)];
    end
end
% Kenneth Marino
