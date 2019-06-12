clc
interior_circle = [970.098904698310,284.110116989635,514.083449970689,30.2380952380952];
PERCENT = 75; %in [0 100]
SKIPSTEP = 20;
CENTERX = interior_circle(1,1);
CENTERY = interior_circle(1,2);
TIMESTEP = 1/102;

%STEP 1: CHOOSE ONLY INTERESTING TRAJECTORIES
tr2 = compute_new_tr(tr, data_tracking, PERCENT);
tr3 = compute_velocity(tr2, TIMESTEP, CENTERX, CENTERY, SKIPSTEP);
data_tracking2 = get_data_tracking(set,tr3);

%STEP 2: COMPUTE VELOCITIES
NB_LAYER_IN_SB = 5; %nb of layers or r which separate the shear cell in the shear band
NB_LAYER_OUT_SB = 10; %nb of layers or r which separate the shear cell out of the shear band
tr4 = sortrows(tr3,7);
s_tr4 = size(tr4);
% %tr4 = smooth_pos(tr4);
% vr = smoothn(tr4(:,5),'robust');
% vtheta = smoothn(tr4(:,6),'robust');
% tr4(:,5) = vr;
% tr4(:,6) = vtheta;
% v_mean = mean_velocity(R_LAYER_WIDE,tr4);
r_min = tr4(1,7);
r_max = tr4(s_tr4(1),7);
regions = [linspace(r_min, r_min + 100, NB_LAYER_IN_SB),linspace(r_min + 100, r_max, NB_LAYER_OUT_SB)];
regions = [regions(1:NB_LAYER_IN_SB),regions(NB_LAYER_IN_SB+2:end)];
Vel_reg = Vel_region_time(regions,tr4);
%% 
%STEP 3: DISPLAY VELOCITIES
PIXEL_SCALE = 5.13;
R_LAYER_WIDE = 30;
PICTURE_BAND = 150;
display_velocity(tr4,v_mean,PIXEL_SCALE);
%display_velocity_in_time(tr3,PIXEL_SCALE,PICTURE_BAND,R_LAYER_WIDE);
%display_Vel_region_time(Vel_reg);
%display_Vel_error_bars(Vel_reg);

%STEP 4: CLEAR THE WORKSPACE PLEASE TWROW THE USELESS DATA/VARIABLES
clear PIXEL_SCALE R_LAYER_WIDE PERCENT SKIPSTEP CENTERX CENTERY PICTURE_BAND TIMESTEP 
clear s_tr4 regions r_min r_max NB_LAYER_IN_SB NB_LAYER_OUT_SB