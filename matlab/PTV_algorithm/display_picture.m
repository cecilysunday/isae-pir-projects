% This function displays the picture nb "num" in the set of pictures "set".
% Data is the array containing info about the detection (see 
% parameters_tracking)
%
% If mode = 0: the unfiltered picture is displayed
% If mode = 1: the filtered picture is displayed

function display_picture(set, num, data, mode)

pict = read(set,num);

if (mode == 0)
    imshow(pict); axis on;
end
if (mode == 1)
    pict2 = treat_image(pict, data);
    imshow(pict2); axis on;
end

hold on;

end