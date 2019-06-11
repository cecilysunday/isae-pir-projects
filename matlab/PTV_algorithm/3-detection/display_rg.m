% INPUT: 
% set = set of pictures
% num = ID of the picture you want to display
% data = data used for detecting particles (see parameters_tracking);
% array = data provided by the detect_particle function 
% istreated: set the value to 1 if you want to display the blobs on the
% treated/filtered image; otherwise set this value to 0
%
% This function plots the detected particles according to their gyroradius
% (radius of the particle) and their brightness
%
% OUTPUTs: 2 figures:
%   -figure 1: image with the detected blobs, displayed according to their
%   size (gyroradius)
%   -figure 2: graph showing the detected blobs with a color code
%   corresponding to their size (gyroradius)
%   -figure 3: graph showing the detected blobs with a color code
%   corresponding to their brightness 

function display_rg(set, num, data, array, istreated)

if (istreated == 0)
    pict2 = read(set,num);
elseif (istreated == 1)
    pict = read(set,num);
    pict2 = treat_image(pict, data);
end

imshow(pict2); axis on;
hold on;

sz=size(array);
for i=1:sz(1)
    th = 0:pi/50:2*pi;
    xunit = sqrt(array(i,4)) * cos(th) + array(i,1);
    yunit = sqrt(array(i,4)) * sin(th) + array(i,2);
    plot(xunit, yunit, 'b', 'LineWidth', 2);
end

figure; 
title('radius');
hold on;
axis ij equal;
for i=1:sz(1)
    scatter(array(i,1), array(i,2), array(i,4), array(i,4),'filled', 'LineWidth', 2);
end

figure; 
title('brightness');
hold on;
axis ij equal;
for i=1:sz(1) 
    scatter(array(i,1), array(i,2), array(i,3)*10^-2, array(i,3),'filled', 'LineWidth', 2);
end