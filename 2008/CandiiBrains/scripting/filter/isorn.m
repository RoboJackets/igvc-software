function [y,yh]=isorn(fr1)
r=fr1(:,:,1);
g=fr1(:,:,2);
b=fr1(:,:,3);
%% if r>180
    %% Gamma Corrected
    rg=(r.^2.2)*255;
    gg=(g.^2.2)*255;
    bg=(b.^2.2)*255;
    y1=rg>gg+32 & rg>128 & rg<=176;
    y2=gg<144 & rg>176 & rg<=224;
    y3=rg>gg/6+200 & rg>224;    
    %%
%% else
r=r*255;
g=g*255;
b=b*255;

y4=r<100 & r>g;
y5=r>90 & r>1.275.*g & r<200;
y=((y1|y2|y3) & r>180)|y4|y5&g<230&b<240;

%%hsvstuff
frh=rgb2hsv(fr1);
h=frh(:,:,1).*360;
s=frh(:,:,2);

yh=r<230&(h<20|h>300)&s>.25|(r>=230&y);
