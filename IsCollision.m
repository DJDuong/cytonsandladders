%% IsCollision
% This is based upon Lab 5 exercises
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function [result,intersectPlot] = IsCollision(robot,qMatrix,tFace,tVertex, handles)
result = false;
intersectPlot  = plot3(0,0,0,'g*','MarkerSize',1);
set(handles.CollisionStatus,'String', 'Collision Status: Checking trajectory...', 'BackgroundColor','g');
pause(0.5);
for qIndex = 1:size(qMatrix,1)
%     q = qMatrix(qIndex,:);
    
    % Get the transform of every joint (i.e. start and end of every link)  
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
    % Iterate through each stored ply object
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

        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    intersectPlot = plot3(intersectP(1),intersectP(2),intersectP(3),'g*','MarkerSize',20);
                    display('Intersection');
                    set(handles.CollisionStatus,'String', 'Collision Status: Detected and Avoiding', 'BackgroundColor','r');
                    result = true;
                    return;
                end
            end    
        end
    end
end
set(handles.CollisionStatus,'String', 'Collision Status: None Detected', 'BackgroundColor','w');
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
%% GetLinkPoses
function [ transforms ] = GetLinkPoses( q, robot)
%q - robot joint angles
%robot -  seriallink robot model
%transforms - list of transforms

links = robot.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);

    transforms(:,:,i + 1) = current_transform;
end
end