%--------------------------------------------------------------------------
%PARAMETERS FOR BUILDING THE MASK(S)

%x0 = x-center of the ellipses 
x0 = 1000;
x02 = 0;
x03 = 0;

%y0 = y-Center of the ellipses 
y0 = 290; 
y02 = 0;
y03 = 0; 

%e = eccentricity of the ellipses
e = 0.2; 
e2 = 0;
e3 = 0; 

%a = semi-major axis of the intern ellipse 
a = 510; %525
a2 = 0;
a3 = 0;
 
%width = width of the mask 
width = 460;
width2 = 0; 
width3 = 0;
%--------------------------------------------------------------------------
%PARAMETERS USED FOR THE DETECTION 

%blob_diam
%This parameter is used in the bpass function; see "detect_particles".
%the diameter of the 'blob's you want to find in pixels.

%th
%This parameter is used in the pkfnd function; see "detect_particles".
%The minimum brightness of a pixel that might be local maxima. 
%(NOTE: Make it big and the code runs faste but you might miss some 
%particles. Make it small and you'll get everything and it'll be slow)

%sz
%This parameter is used in the pkfnd function; see "detect_particles". 
%Roughly the diameter of the average feature to look for in pixels. 
%This parameter is helpful for noisy data.
%If your data is noisy, (e.g. a single particle has multiple local maxima), 
%then set this optional keyword to a value slightly larger than the diameter 
%of your blob. If multiple peaks are found withing a radius of sz/2 then 
%the code will keep only the brightest. Also gets rid of all peaks within 
%sz of boundary.

%sz2
%This parameter is used in the cntrd function; see "detect_particles".
%Diamter of the window over which to average to calculate the centroid.  
%Should be big enough to capture the whole particle but not so big that it 
%captures others.  
%If initial guess of center (from pkfnd) is far from the centroid, the
%window will need to be larger than the particle size.  
%The RECCOMMENDED size is the long lengthscale used in bpass plus 2.

%brightn_tr
%This parameter is used in "detect_particles".
%Brightness_treshold : seuil de luminosit�. Les points d'int�r�t dont la 
%luminosit� est en dessous de ce seuil ne seront pas d�tect�s. Les points 
%d'int�r�t dont la luminosit� est au dessus de ce seuil seront d�tect�s. 
%Pour d�tecter beaucoup de particules sur une image, mettre la valeur � 0.
%Pour d�tecter peu de particules, augmenter la valeur de ce param�tre

blob_diam = 16;
th = 15;%35
sz = blob_diam; 
sz2 = blob_diam + 3;
brightn_tr = 300;
%--------------------------------------------------------------------------
%PARAMETERS USED FOR THE TRACKING

%Param�tre utilis� dans "track_particles"
%Intervalle de temps entre 2 images successives (unit� au choix, pas
%d'importance). Sera utile seulement lors du calcul des vitesses des
%particules.
deltaT = 1;

%This parameter is used in the track function; see "track_particles".
%An estimate of the maximum distance that a particle would move in a 
%single time interval.
maxdisp = blob_diam/2;

%This parameter is used in the track function; see "track_particles".
%This is the number of time steps that a particle can be 'lost' and then 
%recovered again. If the particle reappears after this number of frames has 
%elapsed, it will be tracked as a new particle. The default setting is 0.
%This is useful if particles occasionally 'drop out' of the data.
mem = 5;

%This parameter is used in the track function; see "track_particles".
%Set this keyword to eliminate all trajectories with fewer than "good" 
%valid positions. This is useful for eliminating very short, mostly 'lost' 
%trajectories due to blinking 'noise' particles in the data stream.
good = 2;

%This parameter is used in the track function; see "track_particles".
%set this keyword to 1 if you don't want any text to be plotted at the end
%of execution of "track"
quiet = 1;
%--------------------------------------------------------------------------
%PERSPECTIVE_CORRECTION
%focal length divided by the Z-coordinate (in the camera coordinates
%system) of the plan we are studying
fdivZ = 0.0427;

%pixel size; it depends on the sensor size (s_s)
p_s = 0.0075; 
