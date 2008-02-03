function out=splitsum(in)
sz=size(in);
out=in;
for r=1:sz(3)
    a=0;
    for n=1:sz(2)
        b=in(1,n,r);
        if (b*a<0||b==0)
            a=0;
        end
        a=a+b;
        out(1,n,r)=a;
    end
end

    