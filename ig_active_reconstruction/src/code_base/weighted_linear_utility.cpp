﻿/* Copyright (c) 2016, Stefan Isler, islerstefan@bluewin.ch
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

#include "ig_active_reconstruction/weighted_linear_utility.hpp"

#include <thread>
#include <iostream>
#include <fstream> // std::ofstream
using namespace std;


namespace ig_active_reconstruction
{

std::vector<world_representation::CommunicationInterface::ViewIgResult>
        IgMetricsPerIt(100);

WeightedLinearUtility::WeightedLinearUtility(double cost_weight)
    : world_comm_unit_(nullptr), robot_comm_unit_(nullptr),
      cost_weight_(cost_weight), iter_count_(0)
{
}

void WeightedLinearUtility::useInformationGain(std::string name, double weight)
{
        information_gains_.push_back(name);
        ig_weights_.push_back(weight);
}

void WeightedLinearUtility::setCostWeight(double weight)
{
        cost_weight_ = weight;
}

void WeightedLinearUtility::setIgRetrievalConfig(
        world_representation::CommunicationInterface::IgRetrievalConfig &config)
{
        ig_retrieval_config_ = config;
}

void WeightedLinearUtility::setWorldCommUnit(
        boost::shared_ptr<world_representation::CommunicationInterface>
                world_comm_unit)
{
        world_comm_unit_ = world_comm_unit;
}

void WeightedLinearUtility::setRobotCommUnit(
        boost::shared_ptr<robot::CommunicationInterface> robot_comm_unit)
{
        robot_comm_unit_ = robot_comm_unit;
}

views::View::IdType
WeightedLinearUtility::getNbv(views::ViewSpace::IdSet &id_set,
                              boost::shared_ptr<views::ViewSpace> viewspace,
                              unsigned int itr_count)
{
        // structure to store received values
        std::vector<double> cost_vector;
        std::vector<double> ig_vector;
        // std::vector<std::string> mm_vectors;
        std::string metric_name;
        double max = 0;

        std::cout << "Current Iteration Count = " << iter_count_ << "\n";

        // Required Map Metrics
        /* TODO: make this part of */
        map_metrics_.clear();
        map_metrics_.push_back("world_stats");
        map_metrics_.push_back("world_entropy");

        double total_cost = 0;
        double total_ig = 0;

        world_representation::CommunicationInterface::MapMetricRetrievalCommand
                mm_command;
        mm_command.metric_names = map_metrics_;
        world_representation::CommunicationInterface::
                MapMetricRetrievalResultSet mm_result;
        world_comm_unit_->computeMapMetric(mm_command, mm_result);
        // std::cout << "*****mm_result.at(0).value" << mm_result.at(0).value
        //<< "\n";
#if 0
/**********************************************************************************************************************/
        //double RearSideVoxelFit = 1 - (a * exp(-b * x) + c);
        double RearSideVoxelFit = 1 - (0.577 * exp(-0.106 * (itr_count + 1)) + 0.248);
        metric_name.assign("RearSideVoxelIg");
        max = RearSideVoxelFit;

        double VGFit = 1 - (0.575 * exp(-0.054 * (itr_count + 1)) + 0.206);
        if (VGFit > max)
        {
                max = VGFit;
                metric_name.assign("VasquezGomezAreaFactorIg");
        }

        //double UnobservedVoxelFit = 1 - (0.565 * exp(-0.062 * itr_count) + 0.229);
        double OcclisionAwareFit = 1 - (0.565 * exp(-0.062 * (itr_count + 1)) + 0.229);        
        if (OcclisionAwareFit > max)
        {
                max = OcclisionAwareFit;
                metric_name.assign("OcclusionAwareIg");
        }
        
        double ProximityFit = 1 - (0.789 * exp(-0.032 * (itr_count + 1)) - 0.016);
        if (ProximityFit > max)
        {
                max = ProximityFit;
                metric_name.assign("ProximityCountIg");
        }

        double RearSideEntropyFit = 1 - (1.057 * exp(-0.048 * (itr_count + 1)) - 0.252);
        if (RearSideEntropyFit > max)
        {
                max = RearSideEntropyFit;
                metric_name.assign("RearSideEntropyIg");
        }

        std::cout<<" itr_count : "<<itr_count <<"   metric_name : "<<metric_name<<std::endl;

        //information_gains_.clear();
        //metric_name.assign("RearSideEntropyIg");
        //information_gains_.push_back(metric_name);
/*****************************************************************************************************************/
#endif
        world_representation::CommunicationInterface::IgRetrievalCommand
                command;
        command.config = ig_retrieval_config_;
        command.metric_names = information_gains_;
        command.iteration_count = iter_count_;
        // cout<<"ig_retrieval_config_ :"<<ig_retrieval_config_<<endl;
        // cout<<"information_gains_ :"<<information_gains_<<endl;


        // receive costs and igs
        for (views::View::IdType &view_id : id_set) {
                views::View view = viewspace->getView(view_id);
                // cout << "\n
                // ###################################################"
                //<< endl;
                // cout << "view_id : " << view_id << endl;
                // cout << "view : " << view << endl;

                robot::MovementCost cost;
                world_representation::CommunicationInterface::ViewIgResult
                        information_gains;

                double cost_val = 0;
                double ig_val = 0;

                // cost
                if (robot_comm_unit_ != nullptr && cost_weight_ != 0) {
                        cost = robot_comm_unit_->movementCost(view);


                        if (cost.exception
                            != robot::MovementCost::Exception::NONE)
                                continue; // invalid view... disregard in
                                          // calculation
                        else
                                cost_val = cost.cost;
                        // cout << "cost_val : " << cost_val << endl;
                }

                // information gain - non multithreaded version
                /*if( world_comm_unit_!=nullptr )
                {
                  command.path.clear();
                  command.path.push_back( view.pose() );

                  world_comm_unit_->computeViewIg(command,information_gains);
                  for( unsigned int i= 0; i<information_gains.size(); ++i )
                  {
                    if( information_gains[i].status ==
                world_representation::CommunicationInterface::ResultInformation::SUCCEEDED
                )
                    {
                      //std::cout<<"\nReturned gain of metric
                "<<i<<":"<<information_gains[i].predicted_gain; ig_val +=
                ig_weights_[i]*information_gains[i].predicted_gain;
                    }
                  }
                  //std::cout<<"\nReturned total information gain is: "<<ig_val;
                }
                total_ig += ig_val;
                ig_vector.push_back(ig_val);
                */
                total_cost += cost_val;
                cost_vector.push_back(cost_val);
        }

        // cout << "total_cost : " << total_cost << endl;

        // multithreaded information gain retrieval
        unsigned int number_of_threads = 8;
        ig_vector.resize(id_set.size(), 0);
        std::vector<double> total_multitthread_ig(number_of_threads, 0);
        std::vector<std::thread> threads;
        for (size_t i = 0; i < number_of_threads; ++i) {
                threads.push_back(std::thread(
                        &WeightedLinearUtility::getIg, this,
                        std::ref(ig_vector), std::ref(total_multitthread_ig[i]),
                        command, std::ref(id_set), viewspace, i,
                        number_of_threads));
        }
        for (size_t i = 0; i < number_of_threads; ++i) {
                threads[i].join();
                total_ig += total_multitthread_ig[i];
                // cout << "ig :" << i << "gain :" << total_multitthread_ig[i]
                // << endl;
        }

        // cout << "total_ig : " << total_ig << endl;

        // The answer ........... Plot this and print this!!!
        std::ofstream ofs;
        ofs.open("rabbit_weighted_gain.csv",
                 std::ofstream::out | std::ofstream::app);
        ofs << total_ig << ", ";

        // calculate utility and choose nbv
        views::View::IdType nbv;
        double best_util = std::numeric_limits<double>::lowest();

        double cost_factor;

        if (total_ig == 0)
                total_ig = 1;

        if (total_cost == 0)
                cost_factor = 0;
        else
                cost_factor = cost_weight_ / total_cost;

        for (unsigned int i = 0; i < id_set.size(); ++i) {
                double utility =
                        ig_vector[i] / total_ig - cost_factor * cost_vector[i];
                /*std::cout << "\n utility of view " << id_set[i] << ": "
                          << utility;
                std::cout << "view :" << id_set[i] << endl;
                std::cout << "total Ig for this view  :" << ig_vector[i]
                          << endl;*/
                if (utility > best_util) {
                        best_util = utility;
                        nbv = id_set[i];
                }
        }
        std::ofstream nbvMetrics;
        nbvMetrics.open("nbvmetrics.txt",
                        std::ofstream::out | std::ofstream::app);

        for (unsigned int i = 0; i < IgMetricsPerIt[nbv].size(); i++) {
                nbvMetrics << IgMetricsPerIt[nbv][i].predicted_gain << ", ";
        }

        iter_count_++;
        nbvMetrics << "\n";

        std::cout << "\n Choosing view " << nbv << ".";
        return nbv;
}

void WeightedLinearUtility::getIg(
        std::vector<double> &ig_vector, double &total_ig,
        world_representation::CommunicationInterface::IgRetrievalCommand
                command,
        views::ViewSpace::IdSet &id_set,
        boost::shared_ptr<views::ViewSpace> viewspace, unsigned int base_index,
        unsigned int batch_size)
{

        // cout << "\n ------------------------------------------------" <<
        // endl;

        // information gain
        if (world_comm_unit_ != nullptr) {
                for (size_t i = base_index; i < id_set.size();
                     i += batch_size) {
                        views::View view = viewspace->getView(id_set[i]);
                        // cout << "calculating ig for view :" << view << endl;

                        world_representation::CommunicationInterface::
                                ViewIgResult information_gains;
                        double ig_val = 0;

                        command.path.clear();
                        command.path.push_back(view.pose());

                        world_comm_unit_->computeViewIg(command,
                                                        information_gains);

                        IgMetricsPerIt[i] = information_gains;

                        for (unsigned int i = 0; i < information_gains.size();
                             ++i) {
                                if (information_gains[i].status
                                    == world_representation::
                                               CommunicationInterface::
                                                       ResultInformation::
                                                               SUCCEEDED) {
                                        std::ofstream outfile;
                                        outfile.open(
                                                "Predicted_ig.txt",
                                                std::ofstream::out
                                                        | std::ios_base::app);

                                        outfile << "information_gains[" << i
                                                << "] :"
                                                << information_gains[i]
                                                           .predicted_gain
                                                << endl;
                                        outfile << "ig metric weight :"
                                                << ig_weights_[i] << endl;

                                        ig_val += ig_weights_[i]
                                                  * information_gains[i]
                                                            .predicted_gain;
                                }
                        }
                        total_ig += ig_val;
                        ig_vector[i] = ig_val;
                        /*cout << "view :" << view << endl;
                        cout << "total ig for this view  :" << ig_vector[i]
                             << endl;
                        cout << "communilate IG for all views :" << total_ig
                             << endl;*/
                }
                return;
        } else
                return;
}


} // namespace ig_active_reconstruction
