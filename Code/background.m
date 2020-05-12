function surface = background(image, x, y, z)
   
    img = imread(image); 
    surface = surf(x,y,z,'CData',img,'FaceColor','texturemap');

end