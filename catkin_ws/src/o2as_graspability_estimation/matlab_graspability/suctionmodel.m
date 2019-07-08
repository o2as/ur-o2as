function hm = suctionmodel(R)
% create a suctionmodel
% R: a radius of a suction pad
% hm: suction model

    hm = zeros(2*R+1, 2*R+1);
    for ii = 1:2*R+1
        for jj = 1:2*R+1
            dist = sqrt((ii-R-1)^2+(jj-R-1)^2);
            if dist < R
                hm(ii,jj) = 1;
            end
        end
    end
    
end
