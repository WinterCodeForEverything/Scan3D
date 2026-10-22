# ifndef SCAN3D_FILEIO_OBJ_READER_JUDGE
#define SCAN3D_FILEIO_OBJ_READER_JUDGE

#include<string>

namespace scan3d {
	namespace io {

		class Model;
		class ObjReaderJudge
		{
		public:
			ObjReaderJudge(std::string file_name) :_file_name(file_name) {}
			~ObjReaderJudge() {}

			//bool load_obj2(const std::string &file_name, Model *model);
			bool obj_file_is_valid();
			bool obj_is_mesh();
		private:
			std::string _file_name;
		};
	}
}
#endif //SCAN3D_FILEIO_OBJ_READER_JUDGE