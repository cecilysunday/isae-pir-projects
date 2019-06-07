% INPUTs:
% set: a set of images
% data_mask: the data used to build an elliptical mask
% data_mask = [x0 y0 a width e]
%
% This function creates and plots an elliptical mask of center (x0;y0),
% semi-majr axis a, width 'width' and ecccentricity e.

function mask(set, data_mask)

x0 = data_mask(1,1);
y0 = data_mask(1,2);
a = data_mask(1,3);
width = data_mask(1,4);
e = data_mask(1,5);

theta = 0 : 0.01 : 2*pi;

% Inner ellipse 
x1 = a * cos(theta) + x0;
y1 = (a*sqrt(abs(e*e-1))) * sin(theta) + y0;

% Outer ellipse
x2 = (a+width) * cos(theta) + x0;
y2 =(a*sqrt(abs(e*e-1))+width) * sin(theta) + y0;

% Plotting the mask
pict = read(set,1);
imshow(pict); axis on;
hold on;
plot(x1,y1,'b', 'LineWidth', 2);
plot(x2,y2,'b', 'LineWidth', 2);
plot(x0,y0,'r--o', 'LineWidth', 2);

end