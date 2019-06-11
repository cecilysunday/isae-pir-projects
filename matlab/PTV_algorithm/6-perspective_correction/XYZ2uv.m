function result = XYZ2uv(XYZ_array,data_perspective,set)

fdivZ = data_perspective(1);
p_s = data_perspective(2);

%from camera coordinates to image coordinates
x = XYZ_array(:,1)*fdivZ;
y = XYZ_array(:,2)*fdivZ;
image_coordinates = [x,y];

%Taking into account the pixel size
x = x./p_s;
y = y./p_s;

%from image coordinates to pixel coordinates
%Computing the center of the image:
pict = read(set,1);
sz = size(pict);
centerX = 0.5*sz(2);
centerY = 0.5*sz(1);

u = x + centerX;
v = y + centerY;

result = [u, v];

end