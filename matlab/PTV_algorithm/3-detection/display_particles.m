% Inputs
% set: a set of pictures
% num: the ID of the picture you want to display
% data: the data used for detection; see parameters_tracking 
% array: output of the function: detect_particles 
% mode: see below
% istreated: see below
%
% Display positions given in the array "array".
% The positions are displayed on the picture number "num" in the set of
% pictures "set". Data is the data used for detecting particles (see
% parameters_tracking)
%
% 2 differents modes ara accessibles:
% Mode 1 : every position is displayed in the same time on "pict"
% Mode 2 : every position is displayed one after the other
%
% Set istreated to 1 and the particles will be displayed in the treated
% image; otherwise set this parameter to 0

function display_particles(set, num, data, array, mode, istreated)

if (istreated == 0)
    pict2 = read(set,num);
elseif (istreated == 1)
    pict = read(set,num);
    pict2 = treat_image(pict, data);
end 

imshow(pict2); axis on
hold on;

sz = size(array);
if mode == 1
    for i=1:sz(1,1)
        plot(array(i,1),array(i,2),'b--o', 'LineWidth', 2);
    end
else if mode == 2
    for i=1:sz(1,1)
        plot(array(i,1),array(i,2),'r--o', 'LineWidth', 2);
        pause(2);
        plot(array(i,1),array(i,2),'b--o', 'LineWidth', 2);
    end
end

end