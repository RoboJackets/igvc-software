t = 0:.001:10;
w = 10*(1-exp(-t./2));
fh = fopen('data.txt', 'w');
for pos = 1:length(t)
    fprintf(fh,'time:%f\n',t(pos));
    fprintf(fh,'motor:%f\n',w(pos));
end
fclose(fh);

% Kenneth Marino
