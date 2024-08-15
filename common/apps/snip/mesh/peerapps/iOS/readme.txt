readme.txt:
This file, introduces the contents under this folder.

libwicedmesh:
The static library project to generate libwicedmesh.a files for multiple architectures and platforms.
It wraps the functions of all common/libraries/mesh_*_lib, so this folder and project should NOT be shipped with the WICED SDK.
The libwicedmesh/xcoudebuild.sh script file can be used to build all libwicedmesh.a files in one command.

MeshApp:
This folder is the workspace folder for MeshFramework project and MeshApp project.
It contains projects to demo mesh functions and for third-party development references, so they should be shipped with the WICED SDK.
MeshApp/MeshFramework - This folder includes the project of MeshFramework which wrapped all mesh functions and APIs from libwicedmesh.
MeshApp/MeshApp - This folder includes the App project demos the features of mesh and show how to use the mesh APIs defined in MeshFramework.
