function angArr = convert2angular(velArr,r)
% Takes in array with times on first column and velocities on others and
% a vector containing the radii of the wheels and returns same array with 
% angular velocities.
    velocities = velArr(:,2:end);
    % Takes out velocities
    [a, numcol] = size(velocities); 
    angArr = velArr;
    for col = 1:numcol
        cur_vel = velocities(:,col);
        cur_vel = cur_vel ./ r(col);
        % Divides by radius to get radians/s
        angArr(:,1+col) = cur_vel;
    end
end
