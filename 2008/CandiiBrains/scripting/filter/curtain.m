function barim=curtain(orim,whim)

whim=imclose(whim,strel('square',3)); %preclean white image
whim=whim';    %flip
orim=orim';
out=orim;     %initalize output

while(any(orim(:))) %are there any orange regions left
	ind=find(orim,1,'first');   %locate first index of first orange blob, 
                                %coloumnwise
                                
	cr=imfill(~orim,ind)~=~orim;    %fill in blob located above and use 
                                    %binary logic to put into current
                                    %region of intrest(cr)
                                    
    wallmask=any(cr,2);             %find which cols cr intersects
    stcol=find(wallmask,1);         %find start of intersection
    edcol=find(wallmask,1,'last');  %find end of intersection
    
    overburn=floor((edcol-stcol)/4);%how much extra room do we want?
    s=size(whim);                   %find dims
    stcol=max(stcol-overburn,1);    %overburn to first col or min
    edcol=min(edcol+overburn,s(1)); %overburn to last col or max
    
    cr=cr(stcol:edcol,:);           %cut out the proper cols from cr

    cwhim=whim(stcol:edcol,:);      %cut out the proper cols from cr
    
	s=size(cwhim);                  %get dims the lazy way
	buff=cr;                        %load cr into a buffer
    
	for n=1:s(2)-1                  %do curtain fill
		cc=buff(:,n);               %get current row(current col(cc))
		uc=[cc(2:end) ; 0];         %shift left (up)
		dc=[0 ; cc(1:end-1)];       %shift right (down)

        buff(:,n+1)=buff(:,n+1)|((uc|cc|dc)&cwhim(:,n+1));
                                  %^or (the two shifted rows) 
                                  %and then (mask by the white image) 
                                  %and then (or the row, with the next row)
    end
    
	orim(stcol:edcol,:)=orim(stcol:edcol,:)&~cr;%remove cr from orim so we don't rescan
	out(stcol:edcol,:)=out(stcol:edcol,:)|buff; %drop buffer into output
    imshow(out,'notruesize')                    %demo picture
    pause
end
barim=out';                                     %flip
return