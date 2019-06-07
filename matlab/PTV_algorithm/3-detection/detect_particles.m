% INPUTs:
% set = a set of images
% num = the ID of the image you want to display
% data = the data used for detection
% interactive: see below
%
% This function detects the particles which are on the picture no 'num' of
% the image set 'set'
%
% Set interactive to 1 and the detection will be interactive (see the cntrd
% function for more details). Otherwise set interactive to 0
%
%OUTPUT:  a N x 4 array containing, x, y and brightness for each particle
%detected:
%
%out(:,1) is the x-coordinates
%out(:,2) is the y-coordinates
%out(:,3) is the brightnesses
%out(:,4) is the sqare of the radius of gyration

function result = detect_particles(set, num, data, interactive)

    pict = read(set,num);
    pict2 = treat_image(pict, data);
    
    pk = pkfnd(pict2,data(1,2),data(1,3));
    
    if interactive == 0
        cnt = cntrd(pict2,pk,data(1,4));
    elseif interactive == 1
        cnt = cntrd(pict2,pk,data(1,4),1);
    end
    
    cnt2 = [];
    sz = size(cnt);
    j=1;
    for i=1:sz(1,1)
        if cnt(i,3) >= data(1,5)
            cnt2(j,:) = cnt(i,:);
            j = j+1;
        end
    end
    result = cnt2;
end