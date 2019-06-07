% INPUTs: 
% pict: the picture to be treated
% data: the data used for the detection
%
%This function returned a filtered image

function result = treat_image(pict, data)

%pict = 255 - pict; %optional
pict2 = bpass(pict,1,data(1,1));
result = pict2;

end