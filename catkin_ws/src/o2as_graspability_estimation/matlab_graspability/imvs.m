function imvs(im, txt, f_show, f_pause, f_save)

if f_show
    imv(im), title(txt)
    if f_pause
        pause
    end
end
if f_save
   imwrite(im, [txt, '.BMP'], 'BMP');
end