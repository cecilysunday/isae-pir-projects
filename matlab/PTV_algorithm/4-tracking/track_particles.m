% INPUTs
% set:  set of images
% data: data used for the detection (see parameters_tracking)
% data2: data used for the tracking (see parameters_tracking)
% data_cache data to build an elliptical mask(see parameters_tracking)
% isCacheActivated: see below
%
% This function returns an array containing the computed trajectories
% across all the images of the set. 
% isCacheActivated = 0 => all the detected particles are taken into
% account
% isCacheActivated = 1 => the particles under the mask are ignored
%
% Output: an array containing all the computed trajectories, see the
% 'track' function if you want more details about the output 

function result = track_particles(set, data, data2, data_cache, isCacheActivated)
    
    param.mem = data2(1,3);
    param.good = data2(1,4);
    param.quiet = data2(1,5);
    param.dim = 2;

    %Liste contenant toutes les features détectées sur toutes les images, 
    %et le temps correspondant;
    pos_list = []; 
    
    %nb total de particules détectées sur les "num_pict" premières images 
    %(= nombre cumulé)
    total_numb_part = 0;
    
    %Position (ie numéro de la ligne) actuelle de la ligne à remplir dans 
    %pos_list
    current_pos = 1; 

    for num_pict = 1:set.Count
        
        cnt1 = detect_particles(set,num_pict, data,0);
        cnt = cnt1;
        
        if (isCacheActivated == 1)
            cnt2 = keep_shearing_band(cnt1, data_cache);
            cnt = cnt2;
        end
        
       
        dim = size(cnt);
        nb_part = dim(1,1); % nombre total de particules détectées sur l'image "num_pict"
        total_numb_part = total_numb_part + nb_part;
        actual_nb_in_pos = 1;

        for num = current_pos:total_numb_part
            pos_list(num,:) = [cnt(actual_nb_in_pos,1) 
                               cnt(actual_nb_in_pos,2) 
                               data2(1,1)*(num_pict-1)];
                           
            actual_nb_in_pos = actual_nb_in_pos+1;
        end

        current_pos = total_numb_part + 1;

    end

     result = track(pos_list, data2(1,2),param);

end