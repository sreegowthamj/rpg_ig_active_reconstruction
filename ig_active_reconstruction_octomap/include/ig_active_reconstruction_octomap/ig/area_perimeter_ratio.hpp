#pragma once

#include "ig_active_reconstruction_octomap/octomap_information_gain.hpp"

namespace ig_active_reconstruction
{

namespace world_representation
{

namespace octomap
{
/*! Templated class that implements the average entropy information gain metric
 * based on the definition by S. Kriegel et al. (compare to "Efficient
 * next-best-scan planning for autonomous 3d surface reconstruction of unknown
 * objects."), as used in the ICRA paper "An Information Gain Formulation for
 * Active Volumetric 3D Reconstruction".
 */
template <class TREE_TYPE>
class AreaPerimeterRatio : public InformationGain<TREE_TYPE>
{
      public:
        typedef typename InformationGain<TREE_TYPE>::Utils Utils;
        typedef typename InformationGain<TREE_TYPE>::Utils::Config Config;
        typedef typename InformationGain<TREE_TYPE>::GainType GainType;

      public:
        /*! Constructor
         */
        AreaPerimeterRatio(Config config = Config());

        /*! Returns the name of the method.
         */
        virtual std::string type();

        /*! Returns the information gain calculated for all data added so far
         * (For all rays).
         */
        virtual GainType getInformation();

        /*! Clears all ray-specific data, next data will be considered part of
         * new ray.
         */
        virtual void makeReadyForNewRay();

        /*! Tells the metric that the current computation has ended and any next
         * call will refer to a new one - resets all internal data, but not any
         * "external" configuration.
         */
        virtual void reset();

        /*! Includes a measurement on the ray.
         * @param node Octomap node traversed by the ray.
         */
        virtual void includeRayMeasurement(typename TREE_TYPE::NodeType *node);

        /*! Includes the endpoint of a ray.
         * @param node Octomap node at end of ray.
         */
        virtual void
        includeEndPointMeasurement(typename TREE_TYPE::NodeType *node);

        /*! Informs the metric that a complete ray was cast through empty space
         * without retrieving any measurements.
         */
        virtual void informAboutVoidRay();

        /*! Returns the number of processed voxels
         */
        virtual uint64_t voxelCount();

      protected:
        /*! Helper function
         * @param node Octomap node traversed by the ray.
         */
        virtual void includeMeasurement(typename TREE_TYPE::NodeType *node);


      private:
        Utils utils_;  //! Providing configuration and often used tools.
        GainType ig_;  //! Current information gain result.
        double p_vis_; //! Running visibility likelihood along a ray.
                       //! (Representing the visibility likelihood of the next
                       //! voxel.)
        uint64_t voxel_count_; //! Counts the total number of considered voxels
                               //! during the current run.
};
} // namespace octomap

} // namespace world_representation

} // namespace ig_active_reconstruction

#include "../src/code_base/ig/area_perimeter_ratio.inl"
