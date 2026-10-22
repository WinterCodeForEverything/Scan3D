#include <module/fileio/point_cloud_io.h>

#include <module/core/point_cloud.h>
#include <Cpp/lib3mf_implicit.hpp>

namespace scan3d {

	namespace io {

	using namespace Lib3MF;

		bool load_3mf( const std::string& file_name, PointCloud* cloud )
		{
			auto wrapper = CWrapper::loadLibrary();
			PModel model = wrapper->CreateModel();
			PReader reader = model->QueryReader("3mf");
			reader->SetStrictModeActive(false);
			reader->ReadFromFile(file_name);
			for (Lib3MF_uint32 iWarning = 0; iWarning < reader->GetWarningCount(); iWarning++) {
				Lib3MF_uint32 nErrorCode;
				std::string sWarningMessage = reader->GetWarning(iWarning, nErrorCode);
				LOG(ERROR) << "Encountered warning #" << nErrorCode << " : " << sWarningMessage << std::endl;
			}
			PObjectIterator objectIterator = model->GetObjects();
			std::vector<sPosition> vMeshVertice;
			while (objectIterator->MoveNext()) {
				PObject object = objectIterator->GetCurrentObject();
				if (object->IsMeshObject()) {
					PMeshObject meshObject = model->GetMeshObjectByID(object->GetResourceID());
					meshObject->GetVertices(vMeshVertice);
				}
				break;
			}

			for ( int i = 0; i < vMeshVertice.size(); i++ )
			{
				cloud->add_vertex( vec3( vMeshVertice[i].m_Coordinates[0], vMeshVertice[i].m_Coordinates[1], vMeshVertice[i].m_Coordinates[2] ) );
			}

			return true;
		}
	} // namespace io

} // namespace easy3
