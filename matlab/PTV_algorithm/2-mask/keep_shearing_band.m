% INPUTs:
% array: a set of detected particles, provided by the function:
% detect_particles
% data_mask: the data used to build an elliptical mask
%
% This function returns a new array containing only the particles of array
% which are not located under the mask
%
% OUTPUT: the new array 'result'

function result = keep_shearing_band(array,data_mask)

x0 = data_mask(1,1);
y0 = data_mask(1,2);
a = data_mask(1,3);
width = data_mask(1,4);
e = data_mask(1,5);

s = size(array);
result = [];
i2 = 1;

for (i = 1:s(1))
    if (isUnderMask(array(i,1),array(i,2),x0,y0,a,e,width) == false)
        result(i2,:) = array(i,:);
        i2 = i2 + 1;
    end
end

end
