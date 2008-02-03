fr1=double(imread('/home/robo/Desktop/Candii Brains/segmented frames/other/linegoal.bmp'))./255;

r=fr1(:,:,1);
g=fr1(:,:,2);
b=fr1(:,:,3);
goal=fr1(trip(~(r==0&g==0&b==0)));
goal=reshape(goal,3,length(goal)/3);
fr1=double(imread('/home/robo/Desktop/Candii Brains/segmented frames/other/linefail.bmp'))./255;

r=fr1(:,:,1);
g=fr1(:,:,2);
b=fr1(:,:,3);
fail=fr1(trip(~(r==0&g==0&b==0)));
fail=reshape(fail,3,length(fail)/3);

target=[ones(1,length(goal)) fail(1,:).*0];
inp=[goal,fail];