function barim=curtain(orim,whim)
whim=imclose(whim,strel('square',3));
whim=whim';
orim=orim';
out=orim;

while(any(orim(:)))
	ind=find(orim,1,'first');
	cr=imfill(~orim,ind)~=~orim;
    
    wallmask=any(cr,2);
    stcol=find(wallmask,1);
    edcol=find(wallmask,1,'last');
    overburn=floor((edcol-stcol)/4);%how much extra room do we want?
    s=size(whim);
    stcol=max(stcol-overburn,1);
    edcol=min(edcol+overburn,s(1));
    
    cr=cr(stcol:edcol,:);

    cwhim=whim(stcol:edcol,:);
    
	s=size(cwhim);
	buff=cr;
	for n=1:s(2)-1
		cc=buff(:,n);
		uc=[cc(2:end) ; 0];
		dc=[0 ; cc(1:end-1)];
		buff(:,n+1)=buff(:,n+1)|((uc|cc|dc)&cwhim(:,n+1));
    end
	orim(stcol:edcol,:)=orim(stcol:edcol,:)&~cr;
	out(stcol:edcol,:)=out(stcol:edcol,:)|buff;
  %   imshow(out)
  %   pause
end
barim=out';
return