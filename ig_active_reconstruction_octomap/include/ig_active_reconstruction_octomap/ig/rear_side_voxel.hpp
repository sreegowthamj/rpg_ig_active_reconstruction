/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
 * (ETH Zurich / Robotics and Perception Group, University of Zurich,
 * Switzerland)
 *
 * This file is part of ig_active_reconstruction, software for information gain
 * based, active reconstruction.
 *
 * ig_active_reconstruction is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version. ig_active_reconstruction is
 * distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details. Please refer to the GNU Lesser General Public License for details on
 * the license, on <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "ig_active_reconstruction_octomap/octomap_information_gain.hpp"

namespace ig_active_reconstruction
{

namespace world_representation
{

namespace octomap
{
/*! Templated class that implements the "rear side voxel information gain" as
 * presented in the ICRA paper "An Information Gain Formulation for Active
 * Volumetric 3D Reconstruction". It counts how many rays are incident on the
 * rear side of a previously observed surface.
 */
template <class TREE_TYPE>
class RearSideVoxelIg : public InformationGain<TREE_TYPE>
{
      public:
        typedef typename InformationGain<TREE_TYPE>::Utils Utils;
        typedef typename InformationGain<TREE_TYPE>::Utils::Config Config;
        typedef typename InformationGain<TREE_TYPE>::GainType GainType;

      public:
        /*! Constructor
         */
        RearSideVoxelIg(Config config = Config());

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
        Utils utils_; //! Providing configuration and often used tools.
        GainType rear_side_voxel_count_; //! Current information gain result.
        bool previous_voxel_unknown_;
};
} // namespace octomap

} // namespace world_representation

} // namespace ig_active_reconstruction

#include "../src/code_base/ig/rear_side_voxel.inl"