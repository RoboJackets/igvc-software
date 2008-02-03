function out=edgewalk(im,thr)
run=1;
out=im;
imh=im>thr(2);
iml=im>thr(1);
% iml=imopen(iml,strel('rectangle',[2 5]));
% imh=imopen(imh,strel('rectangle',[2 5]));
imh=im.*imh;
buf=iml~=iml;
while (1)
	[v,i]=max(imh(:));
	if(v<thr(2))
			break;
	end
	cr=imfill(~iml,i)~=~iml;
	stc=find(max(cr),1,'first');
	edc=find(max(cr),1,'last');
% 	cr(:,stc)=cr(:,stc)*0+1;
% 	cr(:,edc)=cr(:,edc)*0+1;
	[a,in]=max(im(:,stc:edc).*cr(:,stc:edc));
	if(stc==edc)
		im=im.*~cr;
		imh=imh.*~cr;
		continue;
	end
	%%start walk
	buf(in(1),stc)=1;
	for n=(0:(edc-stc-1))
		this=in(n+1);
		next=in(n+2);
		%buf(in(n+1),stc+n)=1;
		if next>=this
			buf(this:this+floor((next-this)/2),stc+n)=...
				(buf(this:this+floor((next-this)/2),stc+n)).*0+1;
			buf(this+floor((next-this)/2)+1:next,stc+n+1)=...
				(buf(this+floor((next-this)/2)+1:next,stc+n+1)).*0+1;
		elseif (next<this)
			buf(this+floor((next-this)/2):this,stc+n)=...
				(buf(this+floor((next-this)/2):this,stc+n)).*0+1;
			buf(next:this+floor((next-this)/2)-1,stc+n+1)=...
				(buf(next:this+floor((next-this)/2)-1,stc+n+1)).*0+1;
		end
	end
	buf(in(end),edc)=1;
	%%end walk
	im=im.*~cr;
	imh=imh.*~cr;
end
out=buf;