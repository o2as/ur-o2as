function overlaid(im, imm)
% show the overlaided image
% im : base image
% imm : overlaid image
%
% 8/10/2018, Yukiyasu Domae, AIST

    tmp = zeros([size(im), 3]);

    for ii = 1:3
        tmp(:,:,ii) = im;
    end

    tmp(:,:,3) = im + imm*255*2;

    figure, imshow(uint8(255.*tmp./max(tmp(:))))

end