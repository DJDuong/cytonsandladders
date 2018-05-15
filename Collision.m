function isCollision = EndCheckCollisionCartesion(cyton, jointCond, minXYZ, maxXYZ)
    cytonPos = cyton.fkine(jointCond);
    cytonPos = cytonPos(1:3, 4)';
    isCollision = false;
    %checks if end effector is within bounds of a rectangular object
    if (cytonPos(1) >= minXYZ(1) && cytonPos(1) <= maxXYZ(1) ...
     && cytonPos(2) >= minXYZ(2) && cytonPos(2) <= maxXYZ(2) ...
     && cytonPos(3) >= minXYZ(3) && cytonPos(3) <= maxXYZ(3))
        isCollision = true;
    end
end

function isCollision = CheckCollisionCartesion(cyton, jointCond, minXYZ, maxXYZ)
    isCollision = false;
    tr = zeros(4,4,cyton.n+1);
    tr(:,:,1) = cyton.base;
    L = cyton.links;
    %gets translations for each link
    for i = 1 : cyton.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(jointCond(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
    for i = 1 : size(tr,3)-1
        %extracts xyz of each translation
        coodPos = tr(1:3, 4, i)';
        %checks if any links are within bounds of a rectangular object
        if (coodPos(1) >= minXYZ(1) && coodPos(1) <= maxXYZ(1) ...
        && coodPos(2) >= minXYZ(2) && coodPos(2) <= maxXYZ(2) ...
        && coodPos(3) >= minXYZ(3) && coodPos(3) <= maxXYZ(3))
            isCollision = true;
        end
    end
end