function isCollision = CheckCollision(cyton, jointMatrix, vertex, faces, faceNormals)
    isCollision = false;
    tr = zeros(4,4,cyton.n+1);
    tr(:,:,1) = cyton.base;
    L = cyton.links;
    
    %gets translations for each link
    for j = 1 : size(jointMatrix, 1)
        jointCond = jointMatrix(j, :);
        for i = 1 : cyton.n
            tr(:,:,i+1) = tr(:,:,i) * trotz(jointCond(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
        end
        for i = 1 : size(tr,3)-1
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check ~= 0
                    isCollision = true;
                end
            end  
        end
    end
end
%% LinePlaneIntersection
% Given a plane (normal and point) and two points that make up another line, get the intersection
% Check == 0 if there is no intersection
% Check == 1 if there is a line plane intersection between the two points
% Check == 2 if the segment lies in the plane (always intersecting)
% Check == 3 if there is intersection point which lies outside line segment
function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

intersectionPoint = [0 0 0];
u = point2OnLine - point1OnLine;
w = point1OnLine - pointOnPlane;
D = dot(planeNormal,u);
N = -dot(planeNormal,w);
check = 0; %#ok<NASGU>
if abs(D) < 10^-7        % The segment is parallel to plane
    if N == 0           % The segment lies in plane
        check = 2;
        return
    else
        check = 0;       %no intersection
        return
    end
end

%compute the intersection parameter
sI = N / D;
intersectionPoint = point1OnLine + sI.*u;

if (sI < 0 || sI > 1)
    check= 3;          %The intersection point  lies outside the segment, so there is no intersection
else
    check=1;
end
end