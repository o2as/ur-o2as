function dst = fmake(im, ws)

    dst = zeros(size(im,1)+ws*2, size(im,2)+ws*2);
    
    dst(ws+1:end-ws, ws+1:end-ws) = im;

end