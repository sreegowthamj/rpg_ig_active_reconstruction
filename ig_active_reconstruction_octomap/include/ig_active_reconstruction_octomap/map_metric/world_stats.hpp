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

#include "ig_active_reconstruction_octomap/octomap_map_metric.hpp"
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "ig_active_reconstruction_octomap/octomap_information_gain.hpp"

namespace ig_active_reconstruction
{

namespace world_representation
{

namespace octomap
{

/*! Class that calculates a whole set of tree metrics simultaneously.
 *
 * TODO: refactoring...
 */
using namespace std;

template <class TREE_TYPE> class WorldStats : public MapMetric<TREE_TYPE>
{
      public:
        WorldStats()
            : unknown_voxel_count(0), occupied_voxel_count(0),
              free_voxel_count(0), occupied_voxel_area(0), free_voxel_area(0)
        {
                std::cout << "World_stats constructor \n";
        }
        virtual std::string type()
        {
                ROS_INFO(
                        "Registered*****************************************\n");
                return "world_stats";
        }

        virtual typename MapMetric<TREE_TYPE>::Result
        calculateOn(boost::shared_ptr<TREE_TYPE> octree)
        {
                // MapMetric<TREE_TYPE>::Result x;

                for (typename TREE_TYPE::iterator it = octree->begin_leafs(),
                                                  end = octree->end_leafs();
                     it != end; ++it) {

                        double current_voxel_size = it.getSize();

                        if (octree->isNodeOccupied(*it)) {
                                occupied_voxel_count++;
                                occupied_voxel_area = occupied_voxel_area
                                                      + (current_voxel_size
                                                         * current_voxel_size);
                        } else {
                                free_voxel_count++;
                                free_voxel_area = free_voxel_area
                                                  + (current_voxel_size
                                                     * current_voxel_size);
                        }
                }

                std::ofstream outfile;
                outfile.open("occupied_voxel_area.csv",
                             std::ofstream::out | std::ios_base::app);
                outfile << occupied_voxel_area << ", ";

                std::ofstream outfile1;
                outfile1.open("free_voxel_area.csv",
                              std::ofstream::out | std::ios_base::app);
                outfile1 << free_voxel_area << ", ";

                std::ofstream outfile2;
                outfile2.open("known_area.csv",
                              std::ofstream::out | std::ios_base::app);
                outfile2 << (free_voxel_area + occupied_voxel_area) << ", ";

                free_voxel_count = 0;
                occupied_voxel_count = 0;
                occupied_voxel_area = 0;
                free_voxel_area = 0;
                /*TODO: Update this with real value */
                return 0.0;
        }

      private:
        int unknown_voxel_count;
        int occupied_voxel_count;
        int free_voxel_count;
        double occupied_voxel_area;
        double free_voxel_area;

}; // namespace octomap
} // namespace octomap
} // namespace world_representation
} // namespace ig_active_reconstruction

//#include "../../../src/code_base/map_metric/world_stats.inl"
