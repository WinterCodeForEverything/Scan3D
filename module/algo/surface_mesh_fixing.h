#ifndef SCAN3D_ALGO_SURFACE_MESH_FIXING_H
#define SCAN3D_ALGO_SURFACE_MESH_FIXING_H

#include<string>


namespace scan3d {

	class SurfaceMesh;
	//class Basic_TMesh;
    class SurfaceMeshFixing {
    public:
		//SurfaceMeshFixing();
		//~SurfaceMeshFixing();
		SurfaceMesh* removeSelfInteraction( SurfaceMesh *mesh );

	//protected:
	//	T_MESH::Basic_TMesh* convertSurfaceMeshtoTMesh(SurfaceMesh *mesh);
	//	SurfaceMesh* convertTMeshtoSurfaceMesh(T_MESH::Basic_TMesh* mesh);
    };

} // namespace scan3d

#endif  // SCAN3D_ALGO_SURFACE_MESH_FIXING_H