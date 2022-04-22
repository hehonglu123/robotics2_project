function binary_map = colobj2binary_map(colobj, resolution,expansion)

binary_map = binaryOccupancyMap(10,10,resolution);

[X,Y] = meshgrid(0:1/resolution:10,0:1/resolution:10);

% Skip first 4 objects (walls)
for i = 5:length(colobj.obj)
    if colobj.type(i) == 1 % Box
        [xy, xy_ones] = occupancy_rect(X, Y, ...
            colobj.size{i}(1)+expansion, ...
            colobj.size{i}(2)+expansion, ...
            colobj.pos{i}(1), ...
            colobj.pos{i}(2));
        setOccupancy(binary_map, xy, xy_ones);
    elseif colobj.type(i) == 2 % Cylinder
        [xy, xy_ones] = occupancy_circle(X, Y, ...
            colobj.size{i}(1)+expansion, ...
            colobj.pos{i}(1), ...
            colobj.pos{i}(2));
        setOccupancy(binary_map, xy, xy_ones);
    else
        disp('Not supported')
    end
end

% Make walls
ind = (X == min(X(:))) | (X == max(X(:))) | (Y == min(Y(:))) | (Y == max(Y(:)));
x = X(ind);
y = Y(ind);
xy = [x y];
xy_ones = ones(size(x));
setOccupancy(binary_map, xy, xy_ones);

inflate(binary_map, 1/resolution)

end

function [xy, xy_ones] = occupancy_circle(X, Y, r, x_c, y_c)
ind = (X-x_c).^2 + (Y-y_c).^2 <= r^2;
x = X(ind);
y = Y(ind);
xy = [x y];
xy_ones = ones(size(x));
end

function [xy, xy_ones] = occupancy_rect(X, Y, x_w, y_w, x_c, y_c)
ind = abs(X - x_c)*2 < x_w & abs(Y - y_c)*2 < y_w;
x = X(ind);
y = Y(ind);
xy = [x y];
xy_ones = ones(size(x));
end
