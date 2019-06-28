clc
interior_circle = [1016.5 297.5128 512.6698];
R_LAYER_WIDE = 30;
PERCENT = 15; %in [0 100]
SKIPSTEP = 20;
CENTERX = interior_circle(1,1);
CENTERY = interior_circle(1,2);
TIMESTEP = 1/102;
%%
%STEP 1: CHOOSE ONLY INTERESTING TRAJECTORIES
tr2 = compute_new_tr(tr, data_tracking, PERCENT);
disp("tr2 done");
tr3 = compute_velocity(tr2, TIMESTEP, CENTERX, CENTERY, SKIPSTEP);
data_tracking2 = get_data_tracking(set,tr3);
%%
%STEP 2: COMPUTE VELOCITIES
NB_LAYER_IN_SB = 6; %nb of layers or r which separate the shear cell in the shear band
NB_LAYER_OUT_SB = 10; %nb of layers or r which separate the shear cell out of the shear band
tr4 = sortrows(tr3,7);
disp("tr4 done");
s_tr4 = size(tr4);

%tr4 = smooth_pos(tr4);
vr = smoothn(tr4(:,5),'robust');
disp("vr done");
vtheta = smoothn(tr4(:,6),'robust');
disp("vtheta done");
tr4(:,5) = vr;
tr4(:,6) = vtheta;
v_mean = mean_velocity(R_LAYER_WIDE,tr4);
disp("vmean done");

r_min = tr4(1,7);
r_max = tr4(s_tr4(1),7);
regions = [linspace(r_min, r_min + 100, NB_LAYER_IN_SB),linspace(r_min + 100, r_max, NB_LAYER_OUT_SB)];
regions = [regions(1:NB_LAYER_IN_SB),regions(NB_LAYER_IN_SB+2:end)];
Vel_reg = Vel_region_time(regions,tr4);
disp("velreg done");
%% 
%STEP 3: DISPLAY VELOCITIES
PIXEL_SCALE = 5.13; 
R_LAYER_WIDE = 30;
PICTURE_BAND = 150;
PATH = "matlab/Parameters_tests/Repetabilite/";
%display_velocity(tr4,v_mean,PIXEL_SCALE);
display_velocity_in_time(tr3,PIXEL_SCALE,PICTURE_BAND,R_LAYER_WIDE);
%display_Vel_region_time(Vel_reg);
%display_Vel_error_bars(Vel_reg,PIXEL_SCALE);
%profile_for_different_rotation_speed(PIXEL_SCALE,PATH);

%STEP 4: CLEAR THE WORKSPACE PLEASE TWROW THE USELESS DATA/VARIABLES
clear PIXEL_SCALE R_LAYER_WIDE PERCENT SKIPSTEP CENTERX CENTERY PICTURE_BAND TIMESTEP 
clear s_tr4 regions r_min r_max NB_LAYER_IN_SB NB_LAYER_OUT_SB data_tracking data_tracking2 PATH