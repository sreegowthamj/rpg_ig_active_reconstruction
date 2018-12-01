

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

template <class TREE_TYPE> class WorldEntropy : public MapMetric<TREE_TYPE>
{
      public:
        WorldEntropy() : entropy(0), total_vol(0)
        {
                std::cout << "World_Entropy constructor \n";
        }
        virtual std::string type()
        {
                ROS_INFO(
                        "Registered*****************************************\n");
                return "world_entropy";
        }

        virtual typename MapMetric<TREE_TYPE>::Result
        calculateOn(boost::shared_ptr<TREE_TYPE> octree)
        {
                // MapMetric<TREE_TYPE>::Result x;

                for (typename TREE_TYPE::iterator it = octree->begin_leafs(),
                                                  end = octree->end_leafs();
                     it != end; ++it) {

                    double occ = it->getOccupancy();
                    double size = it.getSize();
                    double p_free = 1 - occ;
                    double ent;
                    double vol = (size * size * size);
                    if (occ == 0 || p_free == 0) {
                        ent = 0;
                    }
                    else{
                        ent = -occ * log(occ) - p_free * log(p_free);
                        ent = ent * vol;
                    }
                    entropy = entropy + ent;
                    total_vol = total_vol + vol;
                }

                std::ofstream outfile;
                outfile.open("entropy.csv",
                             std::ofstream::out | std::ios_base::app);
                outfile << (entropy/total_vol) << ", ";

                entropy = 0;
                total_vol = 0;
                /*TODO: Update this with real value */
                return 0.0;
        }

      private:
        double entropy;
        double total_vol;

}; // namespace octomap
} // namespace octomap
} // namespace world_representation
} // namespace ig_active_reconstruction

//#include "../../../src/code_base/map_metric/world_.inl"
