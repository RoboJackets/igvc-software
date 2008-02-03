function out=pedge(im,a)
% finds edges in an image using a edge detector like sobel on gray or color
% images.
% 
% IM is the image
% A is the width of the detector(not implemented fully yet)
% OR is a list of orientations in recursive angle coding (see
% recursive_angle_coding.m). We only support layer 2 for now. 
% 
h=fspecial('sobel');
    


i1=imfilter(im,h);
out=i1;