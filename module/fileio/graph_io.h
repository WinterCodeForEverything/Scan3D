#ifndef SCAN3D_FILEIO_GRAPH_IO_H
#define SCAN3D_FILEIO_GRAPH_IO_H


#include <string>


namespace scan3d {

    class Graph;

    // TODO: add a general purpose file format 'map' capable of storing
    //       point clouds, graphs, and meshes with arbitrary types of
    //       properties (similar to PLY, each property is read/written
    //       as a whole. Check PLY and POLY formats as reference.
    //       When this is done, I don't need BIN and POLY any more).


    class GraphIO
	{
	public:
        // return nullptr if failed. Currently only PLY format is supported.
        static Graph* load(const std::string& file_name);

        // save the graph to a file. return false if failed.
        static bool	save(const std::string& file_name, const Graph* graph);
	};

    namespace io {

        bool load_ply(const std::string& file_name, Graph* graph);
        bool save_ply(const std::string& file_name, const Graph* graph, bool binary = true);

    } // namespace io

} // namespace scan3d

#endif // SCAN3D_FILEIO_GRAPH_IO_H
