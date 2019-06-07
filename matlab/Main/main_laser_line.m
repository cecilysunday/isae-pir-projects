clear, clc
%--------------------------------------------------------------------------
%STEP 1: LOAD MATLAB ENVIRONMENT

addpath('./PTV_algorithm');
addpath('./PTV_algorithm/1-acquisition');
addpath('./PTV_algorithm/2-mask');
addpath('./PTV_algorithm/3-detection');
addpath('./PTV_algorithm/3-detection/sub_programs'); 
addpath('./PTV_algorithm/4-tracking');
addpath('./PTV_algorithm/5-exploitation/interpolation');
addpath('./PTV_algorithm/5-exploitation/data_tracking');
addpath('./PTV_algorithm/5-exploitation/velocity');
addpath('./PTV_algorithm/5-exploitation/laser_profil');
%--------------------------------------------------------------------------
%STEP 2: LOAD PARAMETERS FOR DETECTION (modify the parameters 
%in parameters_tracking.m)
parameters_tracking;
data = [blob_diam th sz sz2 brightn_tr]; %parameters used for detection

%--------------------------------------------------------------------------
%STEP 3: LOAD IMAGES FROM AN INPUT VIDEO OR FROM A COLLECTION OF IMAGES

%directory = fullfile('data/videos');
%videoname = 'Test_jaune_plafond_paslumieres.mp4';
%acquisition(fullfile(directory, videoname));
set = imageSet(fullfile('data\final_test_laser'));
%--------------------------------------------------------------------------
%STEP 4: CREATION OF A MASK (Make sure the mask is OK by printing it with the     
%script below)

%parameters for the rectangular mask:
x0 = 40; 
y0 = 125; 
long = 555; 
larg = 200;

data_mask = [x0 y0 long larg];

%display the picture with your mask
%display_picture(set, 1, data, 0);
%rectangle('Position',[x0 y0 long larg],'EdgeColor','b', 'LineWidth', 2);
%--------------------------------------------------------------------------
%STEP 5: DETECTION OF PARTICLES (go in parameters_tracking.m and set the detection 
%parameters to the desired values. Make sure the detection is OK by 
%detecting particles in the first image of the set and by printing all the 
%detected particles in the image)

%elimination of the particles wich are not under the mask
%array = detect_particles(set, 1, data, 0);
%array = check_rectangular_mask(data_mask, array);

%plot the image with all the detected particles under the rectangular mask
%display_particles(set, 1, data, array, 1, 0); 
%--------------------------------------------------------------------------
%STEP 6: build a profil, correct perspective effects and display it
theta = 0.6435;
[data_profil_proj,data_profil] = build_profil(set, data_mask, data, theta);
%%
%display_laser_profile(set, data_profil_proj, data_profil, data_mask, data, 0.5, set.Count/100);
%display_laser_profile_in_time(set, data_profil_proj, data_mask,set.Count/10);
%%
get_GIF(set,data_profil_proj,data_mask,'radial_profile',set.Count/50);
%%
%--------------------------------------------------------------------------
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