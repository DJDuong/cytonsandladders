function collision = CheckCollision(cyton, jointMatrix, tVertex, tFace)
    collision = false;
    linkTransform = zeros(4,4,cyton.n+1);
    linkTransform(:,:,1) = cyton.base;
    linkNo = cyton.links;
    
    for objectItr=1:size(tVertex,2)
        vertex = tVertex{objectItr};
        faces = tFace{objectItr};
        
        faceNormals = zeros(size(faces,1),3);
        for faceIndex = 1:size(faces,1)
            vert1 = vertex(faces(faceIndex,1)',:);
            vert2 = vertex(faces(faceIndex,2)',:);
            vert3 = vertex(faces(faceIndex,3)',:);
            faceNormals(faceIndex,:) = unit(cross(vert2-vert1,vert3-vert1));
        end
    
        %gets translations for each link
        for positionItr = 1 : size(jointMatrix, 1)
            jointCond = jointMatrix(positionItr, :);
            for linkItr = 1 : cyton.n
                linkTransform(:,:,linkItr+1) = linkTransform(:,:,linkItr) * trotz(jointCond(linkItr)+linkNo(linkItr).offset) * transl(0,0,linkNo(linkItr).d) * transl(linkNo(linkItr).a,0,0) * trotx(linkNo(linkItr).alpha);
            end
            for linkItr = 1 : size(linkTransform,3)-1
                for faceIndex = 1:size(faces,1)
                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,linkTransform(1:3,4,linkItr)',linkTransform(1:3,4,linkItr+1)'); 
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                        collision = true;
                        return
                    end
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
%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end