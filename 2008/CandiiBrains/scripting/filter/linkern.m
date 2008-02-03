function ker=linkern(w,h,b,th)
%creates a kernel representing a linear edge with white above and black
%below, that has y intercept B and angle TH. The kernel is W wide and H
%tall.

ker=linkernfast(w,h,b,th);
%ker=linkernstrong(w,h,b,th);
end

function ker=linkernfast(w,h,b,th) %#ok<DEFNU>
%NOT ALWAYS RIGHT
%creates a kernel representing a linear edge with white above and black
%below, that has y intercept B and angle TH. The kernel is W wide and H
%tall.
ker=zeros(h,w);
m=tan(th);
if abs(m)<.000000001
    m=.000000001;
end
y=@(x)m.*x+b;
for x=1:w
    xl=x-1;
    xu=x;
    yl=y(xl);
    if yl<0
        break
    end
    if yl>=h
        ker(:,x:end)=1;
        break
    end
    yu=y(xu);
    ylf=floor(yl);
    yuf=floor(yu);
    THRESH=.99999999;
    if mod(yl,1)>THRESH
        ylf=ylf+1;
    end
    ylm=yl-ylf;
    yum=yu-yuf;
    p1=[x,ylf+1];
    p2=[x,yuf+1];%%with linkern(5,5,0,pi/4):floor(yu+1)doesn't always=floor(yu)+1
    if yuf==ylf
        ap1=(ylm+yum)*.5;
        ap2=ap1;
        p2=p1;
    else
        if p2(2)>p1(2)
            ap2=yum^2*.5/m;
            ap1=.5*m-ap2+ylm;
        else
            ap1=-.5*ylm^2/m;
            ap2=-.5*m-ap1+yum;
        end
    end
    
    if p1(2)<=h&&p1(2)>=1 %%p1 in bounds(redundant)
        ker(p1(2),x)=ap1;
    end
    if p2(2)<=h&&p2(2)>=1%%p2 in bounds
        ker(p2(2),x)=ap2;
    end
    low=min(p2(2), p1(2));
    if low>1 %% at least one square to fill
        ker(1:low-1,x)=ones(low-1,1);
    end
end
ker=flipud(ker).^.45;
end

function ker=linkernstrong(w,h,b,th) %#ok<DEFNU>
%NOT ALWAYS RIGHT
%creates a kernel representing a linear edge with white above and black
%below, that has y intercept B and angle TH. The kernel is W wide and H
%tall.
ker=zeros(h,w);
m=tan(th);
y=@(x)m.*x+b;
for x=1:w
    xl=x-1;
    xu=x;
    yl=y(xl);
    if yl<0||yl>=h
        break
    end
    yu=y(xu);
    p1=[x,floor(yl)+1];
    if floor(yu)==floor(yl)
        ap1=(mod(yl,1)+mod(yu,1))*.5;
        ap2=ap1;
        p2=p1;
    else
        p2=[x,floor(yu)+1];%%with linkern(5,5,0,pi/4):floor(yu+1)doesn't always=floor(yu)+1
        if p2(2)>p1(2)
            ap2=mod(yu,1)^2*.5/m;
            ap1=.5*m-ap2+mod(yl,1);
        else
            ap1=-.5*mod(yl,1)^2/m;
            ap2=-.5*m-ap1+mod(yu,1);
        end
    end
    
    if p1(2)<=h&&p1(2)>=1 %%p1 in bounds(redundant)
        ker(p1(2),x)=ap1;
    end
    if p2(2)<=h&&p2(2)>=1%%p2 in bounds
        ker(p2(2),x)=ap2;
    end
    low=min(p2(2), p1(2));
    if low>1 %% at least one square to fill
        ker(1:low-1,x)=ones(low-1,1);
    end
end
ker=flipud(ker);
end