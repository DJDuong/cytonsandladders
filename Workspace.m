function InitialiseWorkspace()
%Initial Test, need to add correct locations
%Cage
[Tri,Pts] = plyread('cage.ply','tri');                                     %display cage
trisurf(Tri,Pts(:,1),Pts(:,2),Pts(:,3));                                   %display cage in default location
hold on
%Card reader
[Tri,Pts] = plyread('reader.ply','tri');                                   
trisurf(Tri,Pts(:,1),Pts(:,2),Pts(:,3));
camlight;
%beacon
[Tri,Pts] = plyread('beacon.ply','tri');                                   
trisurf(Tri,Pts(:,1),Pts(:,2),Pts(:,3));
camlight;
%button
[Tri,Pts] = plyread('button.ply','tri');                                   
trisurf(Tri,Pts(:,1),Pts(:,2),Pts(:,3));
camlight;
end