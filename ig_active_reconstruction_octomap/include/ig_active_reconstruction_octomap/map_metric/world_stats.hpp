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
        virtual std::string type()
        {
                ROS_INFO(
                        "Registered*****************************************\n");
                return "world_stats";
        }

        virtual typename MapMetric<TREE_TYPE>::Result
        calculateOn(boost::shared_ptr<TREE_TYPE> octree)
        {
                for (typename TREE_TYPE::iterator it = octree->begin_leafs(),
                                                  end = octree->end_leafs();
                     it != end; ++it) {
                        // manipulate node, e.g.:
                        std::cout << "Node center: " << it.getCoordinate()
                                  << std::endl;
                        std::cout << "Node size: " << it.getSize() << std::endl;
                        std::cout << "Node value: " << it->getValue()
                                  << std::endl;
                }
        }
};

} // namespace octomap

} // namespace world_representation

} // namespace ig_active_reconstruction

//#include "../../../src/code_base/map_metric/world_stats.inl"
