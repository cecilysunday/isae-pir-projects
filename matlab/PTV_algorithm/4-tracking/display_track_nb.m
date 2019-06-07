% INPUTs:
% set: a set of images 
% tr: a set of trajectories, given by the function: track_particles 
% deltaT: the timestep between 2 successives frames
% nbParticle: the ID of the particle you want to display
%
% This function plots the trajectory of the particle 'nbParticle' accross
% all the images of the set, using the data in tr

function display_track_nb(set, tr, deltaT, nbParticle)

sz = size(tr);

if nbParticle > tr(sz(1,1),4)
    disp(['nbParticle incorrect, nbParticle must be between 1 and ', num2str(tr(sz(1,1),4))]);
else
    i=1;
    while (tr(i,4) ~= nbParticle)
        i = i+1;
    end
    while (i<sz(1,1) && tr(i,4) == nbParticle)
        I = read(set,tr(i,3)+1);
        imshow(I); axis on;
        hold on;
        plot(tr(i,1),tr(i,2),'b--o', 'LineWidth', 2);
        i = i+1;
        pause(1);
    end
    I = read(set,tr(i,3)+1);
    imshow(I); axis on;
    hold on;
    plot(tr(i,1),tr(i,2),'b--o', 'LineWidth', 2);  
    pause(1);
end
end