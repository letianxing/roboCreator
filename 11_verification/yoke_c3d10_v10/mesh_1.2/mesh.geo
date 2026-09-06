SetFactory("OpenCASCADE");
v()=ShapeFromFile("G:/Golands/roboCreator/09_cad/ankle_v09/RollTransferYoke.step");
Mesh.MeshSizeMin=1.2; Mesh.MeshSizeMax=1.2;
Mesh.ElementOrder=2; Mesh.SecondOrderLinear=0; Mesh.HighOrderOptimize=1; Mesh.MshFileVersion=2.2; Mesh.SaveAll=1;
