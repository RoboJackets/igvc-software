function showData(arr)
[a, numcols] = size(arr);
for col = 2:numcols
    plot(arr(:,1), arr(:,col));
end
end
% Kenneth Marino

