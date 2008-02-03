clear
for number=19:144
close all
clc
clear ~(n*)
bw1 = imread('circbw.tif');
fr1 = double(imread(['i/' num2str(number)]))./255;
fr1(2:2:end,:,:)=[];
bw1=[bw1 bw1(:,end).*0];
bw1=rot90(bw1);
bw1=[bw1 bw1(:,end).*0];
bw1=rot90(bw1);
bw1=[bw1 bw1(:,end).*0];
bw1=rot90(bw1);
bw1=[bw1 bw1(:,end).*0];
bw1=rot90(bw1);
bw2 = bwperim(bw1);
%imshow(bw1)

r=fr1(:,:,1);
g=fr1(:,:,2);
b=fr1(:,:,3);

%find by green
orn=fr1(:,:,1)-fr1(:,:,2);
%%super good greedy red
%isgl=(and((orn>0.05),(fr1(:,:,2)>30/255)));
%%bright reluctant
isgl3=(orn>0.2) & (fr1(:,:,2)>30/255);

%Dark Reluctant
%orn>3/255,fr1.r<120,fr.r>30
isgl=(orn>10/255) & (fr1(:,:,1)<130/255) & (fr1(:,:,1)>50/255) & (fr1(:,:,2)<65/255);
isgl2=and(isgl,(fr1(:,:,2)>50/255));
%VeryDark semi-reluctant 
isgl=(orn>7/255) & (fr1(:,:,1)<130/255) & (fr1(:,:,1)>47/255) & (fr1(:,:,2)<45/255);
isgl=isgl&(fr1(:,:,3)<45/255);
isgl=isgl|isgl2|isgl3;
idg=fr1.*trip(isgl)+fr1*.2;



%find by blue
orn=fr1(:,:,1)-fr1(:,:,3);
isbl=(and((orn>.3),(fr1(:,:,2)>30/255)));
idb=fr1.*trip(isbl)+fr1*.2;

%whitish?
bgl=clm(and((fr1(:,:,3)./fr1(:,:,2))>.7,fr1(:,:,3)>.4));
bg=light(fr1,bgl);
%figure,imshow(max(min(or,1),0))
%figure, imshow(imfill(bw2,'holes'))

size(isgl)
size(bgl)

bar=fr1;
bar=light(fr1,or(isgl,bgl));

%figure,imshow(idb)
figure,imshow(fr1)

figure,imshow(bg)
figure,imshow(idg)
%figure, imshow(bar)

%axis equal
pause
end