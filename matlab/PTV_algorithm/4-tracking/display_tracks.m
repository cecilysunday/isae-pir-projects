% INPUTS
% set: a set of images
% tr: a set of trajectories, given by the function: track_particles
% deltaT: timestep between 2 successives frames
%
% This function displays all the computed tracks in tr, accross the images
% from the set 'set'

function display_tracks(set, tr, deltaT)

tr = sortrows(tr,3);
sz = size(tr);
i = 1;
%Parcours des images du set
while (i < sz(1))
    
    I = read(set,tr(i,3)+1);
    imshow(I); axis on;
    hold on;
    
    nbPicture = tr(i,3);
    while(i < sz(1) && tr(i,3) == nbPicture)
        plot(tr(i,1),tr(i,2),'b--o', 'LineWidth', 2);
        i = i+1;
    end
      pause(1);
end
I = read(set,tr(i,3));
imshow(I); axis on;
hold on;
plot(tr(i,1),tr(i,2),'b--o', 'LineWidth', 2);
pause(1);
end