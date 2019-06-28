clear, clc
%%
%STEP 1: LOAD MATLAB ENVIRONMENT

addpath('matlab/Main');
addpath('matlab/PTV_algorithm');
addpath('matlab/PTV_algorithm/1-acquisition');
addpath('matlab/PTV_algorithm/2-mask');
addpath('matlab/PTV_algorithm/3-detection');
addpath('matlab/PTV_algorithm/3-detection/sub_programs'); 
addpath('matlab/PTV_algorithm/4-tracking');
addpath('matlab/PTV_algorithm/5-exploitation/interpolation');
addpath('matlab/PTV_algorithm/5-exploitation/data_tracking');
addpath('matlab/PTV_algorithm/5-exploitation/velocity');
addpath('matlab/PTV_algorithm/5-exploitation/laser_profil');
%%
% STEP 2: LOAD PARAMETERS FOR DETECTION (modify the parameters in 
% parameters_tracking.m)
parameters_tracking;
data = [blob_diam th sz sz2 brightn_tr]; %parameters used for detection
%%
%STEP 3: LOAD IMAGES FROM AN INPUT VIDEO OR FROM A COLLECTION OF IMAGES

%directory = fullfile('data/videos');
%videoname = 'Test_jaune_plafond_paslumieres.mp4';
%acquisition(fullfile(directory, videoname));
%set = imageSet(fullfile('matlab/data/nouveau_format/Final_test/70Hz-LaserLine'));
%set = imageSet(fullfile('matlab/data/experiences/Calibrage_laser/12'));
my_set = imageSet(fullfile('matlab/data/experiences/Regime_transitoire/Profil_de_hauteur/60Hz'));
%%
%STEP 4: CREATION OF A MASK (Make sure the mask is OK by printing it with the     
%script below)

%parameters for the rectangular mask:
x0 = 310; 
y0 = 285; %(x0,y0) = point en haut à gauche du masque
long = 158; %longueur du masque (en pixel)
larg = 60; %largeur du masque (en pixel)

data_mask = [x0 y0 long larg];
%%
%display the first picture of the set with your mask to check if the mask
%is correct
display_picture(my_set, 1, data, 0);
rectangle('Position',[x0 y0 long larg],'EdgeColor','b', 'LineWidth', 2);
%%
%STEP 5: DETECTION OF PARTICLES (go in parameters_tracking.m and set the detection 
%parameters to the desired values. Make sure the detection is OK by 
%detecting particles in the first image of the set and by printing all the 
%detected particles in the image)

%elimination of the particles wich are not under the mask
num_image = 1;
array = detect_particles(my_set, num_image, data, 0);
array = check_rectangular_mask(data_mask, array);

%plot the first image of the set with all the detected particles which are
%under the rectangular mask, in order to check it
display_particles(my_set, num_image, data, array, 1, 0); 
%display_rg(set,num_image, data, array, 0);
%%
%STEP 6: build a profile of surface over time

%Parameters used to build the profile
theta = 0.6435; %angle of projection (angle of the camera) (rad)
pixel_scale = 1.0499; %(pixels/mm)
zero = 328; %The value (in pixel) which is considered to be height = 0 (so 
            %the origin of the y-coordinates axis)
%%
%Build your profile
[data_profil_proj,data_profil] = build_profil(my_set, data_mask, data, theta, pixel_scale, zero);
%%
%STEP 7: Display your profile to see if it is correct

%Method 1: 
%display_laser_profile(set, data_profil_proj, data_profil, data_mask, data, 0.25, my_set.Count/100, theta, pixel_scale, zero);

%Method 2: display on the same graph different profiles corresponding to
%different times; useful to compare profiles across time
%display_laser_profile_in_time(data_profil_proj,data_mask,200,theta, pixel_scale, zero, 1, my_set.Count);

%Method 3: save the profile in a .gif
%get_GIF(set,data_profil_proj,data_mask,'radial_profile',5, theta, pixel_scale, zero);
%%
%Display several mean profiles on the same graph

%Bead diameter (mm)
b_d = 4;

begin = 0; % first frame to be considered; if you want to start at the frame 1, please set begin to 0 (not 1)
limit = 500; %last frame to be considered
step = 100; % the step for averaging

finish = begin + step;

leg = {};
s_leg = 1;

for i = begin/step:(limit-step)/step
    limit_inf = begin;
    if begin == 0
        limit_inf = 1;
    end
    mean_profile = compute_average_profile(data_profil_proj, limit_inf, finish);
    s1 = num2str(begin/100);
    s2 = num2str(finish/100);
    s3 = 'to';
    s4 = ' ';
    s5 = 's';
    leg{s_leg} = strcat([s1 s5 s4 s3 s4 s2 s5]);
    s_leg = s_leg + 1;
    plot(mean_profile(:,1), mean_profile(:,2));
    hold on

    xlim([0/b_d 95/b_d]);
    ylim([-data_mask(1,2)-data_mask(1,4)+zero, -data_mask(1,2)+zero]*(cos(theta)/pixel_scale));
    xlabel('bead diameter'); 
    ylabel('height of the surface (mm)');
    set(gca,'DataAspectRatio',[1,2,1]);
    legend(leg);
    suptitle('Mean radial profile of the surface');
    
    begin = finish;
    finish = begin + step;
end
%%
%STEP 7: CLEAR THE WORKSPACE PLEASE TWROW THE USELESS DATA/VARIABLES

clear data_mask x0 y0 a width e
clear data_mask2 x02 y02 a2 width2 e2
clear data_mask3 x03 y03 a3 width3 e3
clear data blob_diam th sz sz2 brightn_tr
clear data2 deltaT maxdisp mem good quiet
clear directory videoname
clear xo yo long larg data_mask pict
clear array 
clear fdivZ p_s theta
%--------------------------------------------------------------------------