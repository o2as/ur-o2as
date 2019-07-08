function imv(im)
    im = double(im);
    imt = uint8(im/max(im(:))*255);
    figure, imshow(imt);
end