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
              free_voxel_count(0),occupied_voxel_volume(0), free_voxel_volume(0)
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
                typename ig_active_reconstruction::world_representation::
                        octomap::InformationGain<TREE_TYPE>::Utils::Config
                                config_;
                typename ig_active_reconstruction::world_representation::
                        octomap::InformationGain<TREE_TYPE>::Utils utils_(
                                config_);


                for (typename TREE_TYPE::iterator it = octree->begin_leafs(),
                                                  end = octree->end_leafs();
                     it != end; ++it) {

                        /*double x = it.getX();
                        double y = it.getY();
                        double z = it.getZ();
                        double current_voxel_size = octree.getMetricSize(x, y, z);*/
                        double current_voxel_size = it.getSize();

                        if (octree->isNodeOccupied(*it)) {
                                occupied_voxel_count++;                                
                                occupied_voxel_volume = occupied_voxel_volume + pow(current_voxel_size, 3);                              
                        } else {
                                free_voxel_count++;                                
                                free_voxel_volume = free_voxel_volume + pow(current_voxel_size, 3);
                        }


                        // std::cout << "Node center: " <<
                        // it.getCoordinate()
                        //          << std::endl;
                        // std::cout << "Node size: " << it.getSize() <<
                        // std::endl; std::cout << "Node value: " <<
                        // it->getValue()
                        //          << std::endl;

                        /*TODO: Update this with real value */
                }

                std::ofstream outfile;
                outfile.open("map_metric_log.txt",
                             std::ofstream::out | std::ios_base::app);

                outfile << "free voxel count = " << free_voxel_count
                        << " occupied_voxel_count = " << occupied_voxel_count
                        << "\n";

                std::ofstream outfile1;
                outfile1.open("voxel_volume_log.txt",
                             std::ofstream::out | std::ios_base::app);
                outfile1 << "free voxel volume = " << free_voxel_volume 
                         << " occupied voxel volume = "<<occupied_voxel_volume
                         << "\n";


                free_voxel_count = 0;
                occupied_voxel_count = 0;
                occupied_voxel_volume = 0;
                free_voxel_volume = 0;
                return 0.0;
        }

      private:
        int unknown_voxel_count;
        int occupied_voxel_count;
        int free_voxel_count;
        double occupied_voxel_volume;
        double free_voxel_volume;

}; // namespace octomap
} // namespace octomap
} // namespace world_representation
} // namespace ig_active_reconstruction

//#include "../../../src/code_base/map_metric/world_stats.inl"
