function imv2(im, minV, maxV)
    im = double(im);
    im = (im<=minV).*minV+im.*(im>minV);
    im = (im>=maxV).*maxV + im.*(im<maxV);
    imt = uint8((im-min(im(:)))/(max(im(:))-min(im(:)))*255);
    figure, imshow(imt);
end