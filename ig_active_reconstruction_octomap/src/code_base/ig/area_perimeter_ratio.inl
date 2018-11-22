#define TEMPT template <class TREE_TYPE>
#define CSCOPE AreaPerimeterRatio<TREE_TYPE>
#include <iostream>
#include <fstream>

namespace ig_active_reconstruction
{

namespace world_representation
{

namespace octomap
{
TEMPT
CSCOPE::AreaPerimeterRatio(Config utils)
    : utils_(utils), voxel_count_(0), ig_(0), p_vis_(1)
{
}

TEMPT
std::string CSCOPE::type()
{
        return "AreaPerimeterRatio";
}

TEMPT
typename CSCOPE::GainType CSCOPE::getInformation()
{
        std::ofstream outfile;
        outfile.open("unobserved_voxel.txt",
                     std::ofstream::out | std::ios_base::app);
        outfile << "\n unobserved_voxel: ig_:" << ig_;

        return ig_;
}

TEMPT
void CSCOPE::makeReadyForNewRay()
{
        p_vis_ = 1;
}

TEMPT
void CSCOPE::reset()
{
        ig_ = 0;
        p_vis_ = 1;
        voxel_count_ = 0;
}

TEMPT
void CSCOPE::includeRayMeasurement(typename TREE_TYPE::NodeType *node)
{
        includeMeasurement(node);
}

TEMPT
void CSCOPE::includeEndPointMeasurement(typename TREE_TYPE::NodeType *node)
{
        includeMeasurement(node);
}

TEMPT
void CSCOPE::informAboutVoidRay()
{
        voxel_count_ += utils_.config.voxels_in_void_ray;
        // no approximation needed, can be exactly calculated using the
        // geometric series formula
        ig_ += utils_.entropy(utils_.config.p_unknown_prior)
               / (1
                  - utils_.config
                            .p_unknown_prior); // information in void ray...
}

TEMPT
uint64_t CSCOPE::voxelCount()
{
        return voxel_count_;
}

TEMPT
void CSCOPE::includeMeasurement(typename TREE_TYPE::NodeType *node)
{
        double p_occ = utils_.pOccupancy(node);

        if (utils_.isUnknown(p_occ)) {
                ++voxel_count_;
                double vox_ent = utils_.entropy(p_occ);
                ig_ += p_vis_ * vox_ent;
        }

        p_vis_ *= p_occ;
}

} // namespace octomap

} // namespace world_representation

} // namespace ig_active_reconstruction

#undef CSCOPE
#undef TEMPT
