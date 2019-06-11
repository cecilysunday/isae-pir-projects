clear,clc
%--------------------------------------------------------------------------
%STEP 1: LOAD MATLAB ENVIRONMENT

addpath('matlab/Main');
addpath('matlab/PTV_algorithm');
addpath('matlab/PTV_algorithm/1-acquisition');
addpath('matlab/PTV_algorithm/2-mask');
addpath('matlab/PTV_algorithm/3-detection');
addpath('matlab/PTV_algorithm/3-detection/sub_programs'); 
addpath('matlab/PTV_algorithm/4-tracking');
addpath('matlab/PTV_algorithm/5-exploitation');
addpath('matlab/PTV_algorithm/5-exploitation/interpolation');
addpath('matlab/PTV_algorithm/5-exploitation/data_tracking');
addpath('matlab/PTV_algorithm/5-exploitation/velocity');
addpath('matlab/PTV_algorithm/5-exploitation/laser_profil');
addpath('matlab/PTV_algorithm/6-perspective_correction');
%--------------------------------------------------------------------------
%STEP 2: LOAD PARAMETERS FOR DETECTION AND TRACKING (modify the parameters 
%in parameters.m)
%%
parameters_tracking;
data_mask = [x0,y0,a,width,e]; %parameters used for building a mask
data_mask2 = [x02,y02,a2,width2,e2]; %parameters used for building another mask
data_mask3 = [x03,y03,a3,width3,e3]; %parameters used for building another mask
data = [blob_diam th sz sz2 brightn_tr]; %parameters used for detection
data2 = [deltaT maxdisp mem good quiet]; %parameters used for tracking
data_perspective = [fdivZ p_s];
%%
%--------------------------------------------------------------------------
%STEP 3: LOAD IMAGES FROM AN INPUT VIDEO OR FROM A COLLECTION OF IMAGES

%directory = fullfile('data/videos');
%videoname = 'Test_jaune_plafond_paslumieres.mp4';
%acquisition(fullfile(directory, videoname));
set = imageSet(fullfile('matlab/data/nouveau_format/Final-65Hz/Rempli-65Hz'));
%--------------------------------------------------------------------------
%%
%STEP 4: CREATION OF A MASK (go in parameters.m and set the mask parameters 
%to the desired values. Make sure the mask is OK by printing it with the     
%mask function below)

%mask(set, data_mask);
%%
%--------------------------------------------------------------------------
%STEP 5: DETECTION OF PARTICLES (go in parameters.m and set the detection 
%parameters to the desired values. Make sure the detection is OK by 
%detecting particles in the first image of the set and by printing all the 
%detected particles in the image)

%array = detect_particles(set, 1, data, 0);
%display_particles(set, 1, data, array, 1, 1);
%display_rg(set,1, data, array, 1);
%test_sub_pixel_accuracy(array, 20);

%Then test your mask !

array2 = keep_shearing_band(detect_particles(set, 1, data,0), data_mask);
%display_particles(set, 1, data, array2, 1,1);
%display_rg(set, 1, data, array2,1);

array_filtered = filter_size_bright(array2, 10000, 'brightness');
%array_filtered = filter_size_bright(array_filtered, 50, 'radius');
%display_rg(set, 1, data, array_filtered,1);

%test_sub_pixel_accuracy(array2, 20);
%%
%--------------------------------------------------------------------------
%STEP 6: TRACKING OF PARTICLES (go in parameters.m and set the tracking 
%parameters to the desired values. Make sure the tracking is OK by 
%tracking particles in your set of images and by printing all the 
%trajectories computed, or by ploting the trajectories data with the 
%function: get_data_tracking

%TRACKING:
%tr = track_particles(set, data, data2, data_mask, 0); %track without mask
%tr = track_particles(set, data, data2, data_mask, 1); %track with mask

%Correcting perspective view
%tr = [uv2XYZ(tr_0, data_perspective, set), tr_0(:,3:4)];

%GET DATA TRACKING

%data_tracking = get_data_tracking(set, tr);
%display_data_tracking(set, data_tracking);
%%
%TO DISPLAY ONLY ONE PARTICULAR COMPUTED TRACK:
%deltaT = 1;
%display_track_nb(set, tr, deltaT, 918);

%TO DISPLAY ALL THE COMPUTED TRACKS AT THE SAME TIME:
%display_tracks(set, tr, deltaT);
%%
%TO DISPLAY ALL THE COMPUTED TRACKS ONE AFTER THE OTHER:
% sz = size(tr);
% for i=1:tr(sz(1,1),4)
%     display_track_nb(set, tr, deltaT, i);
% end

%clear tr
%--------------------------------------------------------------------------
%STEP 7: CLEAR THE WORKSPACE PLEASE TWROW THE USELESS DATA/VARIABLES

clear data_mask x0 y0 a width e
clear data_mask2 x02 y02 a2 width2 e2
clear data_mask3 x03 y03 a3 width3 e3
clear data blob_diam th sz sz2 brightn_tr
clear data2 deltaT maxdisp mem good quiet
clear data_perspective fdivZ p_s
clear directory videoname 
clear sz i
clear brightness_threshold radius_threshold thresholds
%--------------------------------------------------------------------------