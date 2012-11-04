function t = getTComplete(filename,radius)
% t = getTComplete(filename, radius)
% For file with only one velocity, t and radius are 1X1 arrays. For
% multiple velocities, radius should be a vector containing the radius of
% each of the motors being tested and t will be a vector containing the t
% for each motor.
arr = readFile(filename);
showData(arr);
arr = convert2angular(arr, radius);
t = getT(arr);
end
% Kenneth Marino
