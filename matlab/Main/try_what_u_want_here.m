%--------------------------------------------------------------------------
addpath('./PTV_algorithm');
addpath('./PTV_algorithm/1-acquisition');
addpath('./PTV_algorithm/2-mask');
addpath('./PTV_algorithm/3-detection');
addpath('./PTV_algorithm/3-detection/sub_programs'); 
addpath('./PTV_algorithm/4-tracking');
addpath('./PTV_algorithm/5-exploitation/interpolation');
addpath('./PTV_algorithm/5-exploitation/data_tracking');
addpath('./PTV_algorithm/5-exploitation/velocity');
addpath('./PTV_algorithm/6-perspective_correction');
%--------------------------------------------------------------------------
% parameters_tracking;
% data_mask = [x0,y0,a,width,e]; %parameters used for building a mask
% data_mask2 = [x02,y02,a2,width2,e2]; %parameters used for building another mask
% data_mask3 = [x03,y03,a3,width3,e3]; %parameters used for building another mask
% data = [blob_diam th sz sz2 brightn_tr]; %parameters used for detection
% data2 = [deltaT maxdisp mem good quiet]; %parameters used for tracking
% data_perspective = [fdivZ p_s];
%--------------------------------------------------------------------------
% set = imageSet(fullfile('data/test'));
% 
% th = 0:pi/50:2*pi;
% xunit = 100 * cos(th) + 118;
% yunit = 100 * sin(th) + 9;
% circle = [xunit.' yunit.'];
% 
% circle_plan = XYZ2uv(circle,data_perspective,set);
% 
% pict = read(set,1);
% imshow(pict); axis on;
% hold on;
% plot(circle_plan(:,1),circle_plan(:,2),'b','LineWidth',2);

%result = uv2XYZ([1117,315;12,54],data_perspective,set);
%result2 = XYZ2uv(result,data_perspective,set);
Vel_r = Vel_region_time(30,tr4);
s = size(Vel_r);
s_v = size(Vel_r{1});
leg = {};
for(j=1:s_v(1))
    V_temp_j = [];
    for (i=1:s(2))
       V_temp(i) = Vel_r{i}(j,2);
    end
    V_smooth_temp = smoothn(V_temp,'robust');
    plot(V_smooth_temp,'x');
    hold on;
    r = num2str(Vel_r{i}(j,3));
    leg{j} = strcat('radius',':',r);
end
legend(leg);



% clear data_mask x0 y0 a width e
% clear data_mask2 x02 y02 a2 width2 e2
% clear data_mask3 x03 y03 a3 width3 e3
% clear data blob_diam th sz sz2 brightn_tr
% clear data2 deltaT maxdisp mem good quiet
% clear data_perspective fdivZ p_s
% clear directory videoname 
% clear sz i