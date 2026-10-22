#include <module/fileio/graph_io.h>

#include <clocale>

#include <module/core/graph.h>
#include <module/util/file_system.h>
#include <module/util/stop_watch.h>
#include <module/util/logging.h>


namespace scan3d {


    Graph* GraphIO::load(const std::string& file_name)
	{
		std::setlocale(LC_NUMERIC, "C");

        Graph* graph = new Graph;
        graph->set_name(file_name);

        StopWatch w;
        bool success = false;

        const std::string& ext = file_system::extension(file_name, true);
        if (ext == "ply")
            success = io::load_ply(file_name, graph);
        else if (ext.empty()){
            LOG(ERROR) << "unknown file format: no extension" << ext;
            success = false;
        }
        else {
            LOG(ERROR) << "unknown file format: " << ext << ". Only PLY format is supported for Graph" << ext;
            return nullptr;
        }

        if (!success || graph->n_vertices() == 0) {
            delete graph;
            return nullptr;
        }

        if (success)
            LOG(INFO) << "graph loaded ("
                      << "#vertex: " << graph->n_vertices() << ", "
                      << "#edge: " << graph->n_edges() << "). "
                      << w.time_string();
        else
            LOG(INFO) << "load graph failed";

        return graph;
	}


    bool GraphIO::save(const std::string& file_name, const Graph* graph)
	{
        if (!graph || graph->n_vertices() == 0) {
            LOG(ERROR) << "graph is null";
			return false;
		}

		StopWatch w;
		bool success = false;

        std::string final_name = file_name;
        const std::string& ext = file_system::extension(file_name, true);

        if (ext == "ply" || ext.empty()) {
            if (ext.empty()) {
                LOG(ERROR) << "No extension specified. Default to ply";
                final_name = final_name + ".ply";
            }
            success = io::save_ply(final_name, graph, true);
        }
		else {
            LOG(ERROR) << "unknown file format: " << ext << ". Only PLY format is supported for Graph";
			success = false;
		}

        if (success) {
            LOG(INFO) << "save model done. " << w.time_string();
            return true;
        }
        else {
            LOG(INFO) << "save model failed";
            return false;
        }
	}


} // namespace scan3d
