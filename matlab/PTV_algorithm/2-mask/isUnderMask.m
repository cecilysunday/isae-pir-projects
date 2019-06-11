% INPUTs:
% x_part & y_part: the coordinates of a particle
% x0 & y0: the coordinates of the center of the interior (and exterior)
% ellipse of the mask
% a: the major semi-axis of the inner ellipse
% e: the eccentricity of the inner ellipse
% width: the width of the mask
%
% This function returns TRUE when the particle is under the mask; otherwise
% it returns FALSE. A particle under the mask will not be taken into
% account during the detection and the tracking.

function result = isUnderMask(x_part,y_part,x0,y0,a,e,width)

isUnderMask = true;

if ((x_part-x0)/a)^2 + ((y_part-y0)/(a*sqrt(abs(1-e*e))))^2 > 1
    if ((x_part-x0)/(a+width))^2 + ((y_part-y0)/(a*sqrt(abs(e*e-1))+width))^2 < 1
    	isUnderMask = false;
    end
end

result = isUnderMask;