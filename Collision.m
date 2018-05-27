function isCollision = CheckCollision(cyton, jointMatrix, object)
    isCollision = false;
    tr = zeros(4,4,cyton.n+1);
    tr(:,:,1) = cyton.base;
    L = cyton.links;
    
    %get object center point and radius
    objectPoint = object.point;
    radius = object.radius;
    %make sure work
    
    %gets translations for each link
    for j = 1 : size(jointMatrix, 1)
        jointCond = jointMatrix(j, :);
        for i = 1 : cyton.n
            tr(:,:,i+1) = tr(:,:,i) * trotz(jointCond(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
        end
        for i = 1 : size(tr,3)-1
            %extracts xyz of each translation
            jointPos = tr(1:3, 4, i)';
            %checks if any links are within bounds of object
            if (radius <= DistanceBetweenPoints(jointPos, objectPoint))
                isCollision = true;
            end
        end
    end
end
function distance = DistanceBetweenPoints(point1, point2)
    distance = sqrt((pt1(1)-pt2(1)).^2+...
                    (pt1(2)-pt2(2)).^2+...
                    (pt1(3)-pt2(3)).^2);
end