% This function creates a set of images extracted from the video input. The
% images are stocked in the directory: data/pictures

function acquisition (video) 

v = VideoReader(video);
i=1;
numFrames = v.NumberOfFrames;
N=numFrames;

while (i <= N) 
    
    image = read(v,i);
    image_rgb = rgb2gray(image);
    
    if (i <10)
        name = ['00', num2str(i), '.tif'];
    else
        name = ['0', num2str(i), '.tif'];   
    end
    
    imwrite(image_rgb, name, 'tif');
    i = i+1;
end

movefile 0* data/pictures;
end