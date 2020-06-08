function [statusFlag, handMesh_h] = jacoMoveTest(qStart,qFinish,robot, startPoints, finishPoints)

statusFlag =0;
steps = 50;
s = lspb(0,1,steps); %Scalar function
qMatrix = nan(steps, 6); %Memory allocation
handLoc = transl(1.5, 0.5, 0.8);
handPose = handLoc;
forwardTR = makehgtform('translate',[0,0.04,0]);

[handMesh, VertexCount, Verts] = PlotCutlery('hand2.ply', handLoc(1,4), handLoc(2,4), handLoc(3,4));
handMesh_h = handMesh;
handVerts = Verts;
handVertexCount = VertexCount;
%
% handFaceNormals = zeros(size(handMesh_h.Faces,1),3);
%     for faceIndex = 1:size(handMesh_h.Faces,1)
%         v1 = handMesh_h.Vertices(handMesh_h.Faces(faceIndex,1)',:);
%         v2 = handMesh_h.Vertices(handMesh_h.Faces(faceIndex,2)',:);
%         v3 = handMesh_h.Vertices(handMesh_h.Faces(faceIndex,3)',:);
%         handFaceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
%     end


for i = 1:3:steps
    qMatrix(i,:) = (1-s(i))*qStart + s(i)*qFinish;
    %transform = robot.model.fkine(qMatrix(i,:));
    robot.model.animate(qMatrix(i,:));
    handPose = handPose*forwardTR;
    updatedPoints = [handPose * [handVerts,ones(handVertexCount,1)]']';
    handMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow();   
    lightCurtainStatus = LaserCollision(startPoints, finishPoints, handMesh_h.Vertices, handMesh_h.Faces, handMesh_h.FaceNormals); 
    if lightCurtainStatus == 1
      
      statusFlag = 1;
      return
              
     end
     
 
    
    
end
  
end
