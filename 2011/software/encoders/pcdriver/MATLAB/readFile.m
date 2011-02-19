function arr = readFile(filename)
% This function will take in a file conataining the data from a motor test
% and convert it to an array of numbers with the first row being the times,
% and each row after being one of the motors being tested.
    fh = fopen(filename, 'r');
    line = fgetl(fh);
    row = 0;
    col = 1;
    arr = [];
    while ischar(line)
        col_pos = find(line == ':');
        type = line(1:col_pos-1);
        % Such as time, l_motor, r_motor et cetera. Really just looking for
        % time.
        data = line(col_pos+1:end);
        data = str2num(data);
        % Gets the actual data from the line
        if strcmp(type, 'time')
            % If it's time, goes to the next row and starts at column 1
            row = row + 1;
            col = 1;
        else
            % Otherwise it just goes one column over
            col = col + 1;
        end
        arr(row, col) = data;
        % Writes the data to the appropriate place
        line = fgetl(fh);
        % Gets next line
    end    
    fclose(fh);
end
% Kenneth Marino
