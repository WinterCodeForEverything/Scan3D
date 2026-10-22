#include <module/core/model.h>


namespace scan3d {

    Model::Model(const std::string& name /* = "unknown" */)
            : name_(name)
            , bbox_known_(false)
            , renderer_(nullptr)
    {
    }


    Model::~Model() {
    }


    const Box3& Model::bounding_box() const {
        if (!bbox_known_) {
            Box3& box = const_cast<Model*>(this)->bbox_;
            box.clear();
            for (const auto& p : points())
                box.add_point(p);

            if (box.is_valid())
                const_cast<Model*>(this)->bbox_known_ = true;
            else
                LOG(WARNING) << "model has no valid geometry";
        }
        return bbox_;
    }


    void Model::update_bounding_box() {
        bbox_known_ = false;
    }

}
