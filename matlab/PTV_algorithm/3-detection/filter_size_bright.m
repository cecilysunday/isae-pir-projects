% This funtion filters the array of detected particles according to the 
% parameter 'param'. It suppress the elements in the array that have a
% parameter 'param' above the threshold value 'value'. 
%
% INPUTs:
% array: the array of detected particles to filter
% value: the threshold value 
% param: the parmeter you want to check. 2 values are possible: 'radius'
% and 'brightness'

function result = filter_size_bright(array, value, param)

if strcmp(param,'radius') == 1    
    index = 4;
end
if strcmp(param,'brightness') == 1
    index = 3;
end

array = sortrows(array,index);

i = 1;
sz = size(array);
while (array(i,index) < value) && (i < sz(1))
    i = i+1;
end
result = array(1:i-1,:);

end