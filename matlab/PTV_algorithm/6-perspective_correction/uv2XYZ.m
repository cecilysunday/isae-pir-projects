% This function convert an array of 2D-points (u,v) (in pixel) into a set of
% 3D points (X,Y,Z)
%
% INPUTs:
% uv_array: an array of 2D pixel (u,v) points
% data_perspective: data used to correct perspective effects (see
% parameters_tracking)
% set: a set of images (needed to have the dim of the images
%
%OUTPUT:
% an array of 3D-points (X,Y); Z is known by the user (see data_perspective
% in parameters_tracking)

function result = uv2XYZ(uv_array,data_perspective,set)

fdivZ = data_perspective(1);
p_s = data_perspective(2);
%from pixel coordinates to image coordinates
%Computing the center of the image:
pict = read(set,1);
sz = size(pict);
centerX = 0.5*sz(2);
centerY = 0.5*sz(1);

x = uv_array(:,1)-centerX;
y = uv_array(:,2)-centerY;

%Taking into account the pixel size
image_coordinates = [x*p_s,y*p_s];

%from image coordinates to camera coordinates
X = image_coordinates(:,1)*(1/fdivZ);
Y = image_coordinates(:,2)*(1/fdivZ);
camera_coordinates = [X,Y];

%returning the result
result = camera_coordinates;

end
