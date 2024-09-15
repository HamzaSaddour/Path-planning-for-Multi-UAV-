function wp=divid_WP(way_points)
x=[];
y=[];
z=[];
for i =1:size(way_points,1)-1
    x=[x;way_points(i:1):1:way_points(i+1,1)];
    y=[y;way_points(i:2):1:way_points(i+1,2)];
    z=[z;way_points(i:3):1:way_points(i+1,3)];
end

end