% INPUTs: 
% array: output from the function detect_particles
% i: discretization of [0 1]
%
% This function realizes a test of sub_pixel accuracy. If the output graphs 
% are 'flat', it mean that there is no pixel biaising during the detection

function test_sub_pixel_accuracy(array, i)

subplot(2,1,1);
hist(mod(array(:,1),1),i);
xlabel('fractional part of x coordinates'); 
ylabel('number of particles'); 

subplot(2,1,2);
hist(mod(array(:,2),1),i);
xlabel('fractional part of y coordinates'); 
ylabel('number of particles'); 

end