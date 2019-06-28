clc
%--------------------------------------------------------------------------
%STEP 0: CHECK YOUR WORKSPACE

%Make sure you have in your workspace:
%  - a variable tr containing the trajectories needed to compute the 
%    angular rotational speed
%  - a variable set containing the images needed
%  - a structure data_tracking containing the data about tr
%--------------------------------------------------------------------------
%STEP 1: LOAD MATLAB ENVIRONMENT
%%
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
addpath('matlab/PTV_algorithm/6-perspective_correction');
%--------------------------------------------------------------------------
%%
%STEP 2: LOAD PARAMETERS FOR DETECTION AND TRACKING (modify the parameters 
%in parameters.m)

parameters_tracking;
data_mask = [x0,y0,a,width,e]; %parameters used for building a mask
data_mask2 = [x02,y02,a2,width2,e2]; %parameters used for building another mask
data_mask3 = [x03,y03,a3,width3,e3]; %parameters used for building another mask
data = [blob_diam th sz sz2 brightn_tr]; %parameters used for detection
data2 = [deltaT maxdisp mem good quiet]; %parameters used for tracking
data_perspective = [fdivZ p_s];
%--------------------------------------------------------------------------
%STEP 3: CALCULATE THE EQUATION OF THE INNER CIRCLE OF THE CYLINDER
%%
%Interpolate a circle and plot it to check if the least-square
%interpolation is correct

%Method 1: statique (detection et interpolation sur chaque image du set)
%[all_circles1,interior_circle1,deltaX1,deltaY1,deltaR1] = calculate_radius_cell(set, data, data_mask, data_perspective);
%figure;
%display_picture(set,1,data,0);
%hold on;
%plot_circle(all_circles1);
%plot_circle(interior_circle);

%Method 2: dynamique (interpolation à partir de trajectoires)
[all_circles2, interior_circle2,deltaX2,deltaY2,deltaR2] = interpolate_average(tr, set, 35);
figure; display_picture(set,1,data,0); hold on;
plot_circle(all_circles2);
%plot_circle(interior_circle2);
%%
%Cercle moyen sur les 2 méthodes
%interior_circle = [interior_circle1;interior_circle2];
%mean_circle(1,:) = mean(interior_circle(:,:));
%plot_circle(mean_circle);
%%
CENTERX = interior_circle2(1,1);
CENTERY = interior_circle2(1,2);
INNER_RADIUS = interior_circle2(1,3);
%%
%--------------------------------------------------------------------------
%STEP 4: CHOOSE ONLY INTERESTING TRAJECTORIES

%Select only the longest trajectories where you can follow a particle
%across lots of images. Use the function "compute_new_tr" to eliminate from
%the set of trajectories tr all the trajectories where the particle has 
%been followed among less than PERCENT/100 of all the images

PERCENT = 35; %in [0 100]
tr2 = compute_new_tr(tr, data_tracking, PERCENT);
data_tracking2 = get_data_tracking(set,tr2);
display_data_tracking(set,data_tracking2);
%%
%STEP 5: COMPUTE THE ANGULAR VELOCITY

TIMESTEP = 1/(102); %The timestep between 2 successive frames
SKIPSTEP = 20;
%%
%Method 1: the angular velocity is computed assuming that every particle
%follows the same mean-circle computed in step 3
tr3 = compute_velocity(tr2, TIMESTEP, CENTERX, CENTERY, SKIPSTEP);
v_ang = tr3(:,6)./INNER_RADIUS;
v_ang_moy = mean(v_ang);
%%
%Method 2: the angular velocity is computed assuming that each particle
%follows a circle based on the interpolation of its trajectory
v_ang2 = compute_ang_velocity(tr2, all_circles2, TIMESTEP, SKIPSTEP);
v_ang_moy2 = mean(v_ang2(:,1));
%% 
%Method 3: 
delta = 0.1;
tr4 = select_radius(tr3,INNER_RADIUS-delta,INNER_RADIUS+delta); 
tr4 = sortrows(tr4,4);
v_ang3 = tr4(:,6)./tr4(:,7);
v_ang_moy3 = mean(v_ang3);
clear delta;
%%
%Method 4: see the documentation of get_data_omega_skipstep; this method
%needs lots of calculations
data_angular = get_data_omega_skipstep(tr2, data_tracking2, TIMESTEP, INNER_RADIUS, CENTERX, CENTERY, -0.0967, 15);
%%
%STEP 8: CLEAR THE WORKSPACE PLEASE TWROW THE USELESS DATA/VARIABLES

clear data_mask x0 y0 a width e
clear data_mask2 x02 y02 a2 width2 e2
clear data_mask3 x03 y03 a3 width3 e3
clear data blob_diam th sz sz2 brightn_tr
clear data2 deltaT maxdisp mem good quiet
clear data_perspective fdivZ p_s
%clear interior_circle
clear INNER_RADIUS PIXEL_SCALE CENTERX CENTERY 
clear PERCENT 
clear TIMESTEP SKIPSTEP
clear i
clear deltaR1 deltaX1 deltaY1 deltaR2 deltaX2 deltaY2
%clear data_tracking data_tracking2 
clear all_circles all_circles2
%--------------------------------------------------------------------------