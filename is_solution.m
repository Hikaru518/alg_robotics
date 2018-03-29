% whether within workspace
endeff_x = T(1,4);
endeff_y = T(2,4);
endeff_z = T(3,4);

if sqrt(endeff_x^2+endeff_y^2)>2739 || endeff_x<-2739*cos(pi/6) || endeff_z>3238 || endeff_z<-721 
    in_workspace = false(1);
    fprintf('Warning: T is not within workspace \n');
elseif joint_angles(1)
    
end