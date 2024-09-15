function collision_free = check_path(path,sv)
collision_free=1;
for i = 1:size(path,1)-1
    MotionValid=isMotionValid(sv,path(i,:),path(i+1,:));
    if (~MotionValid)
        collision_free=0;
        break   
    end
end