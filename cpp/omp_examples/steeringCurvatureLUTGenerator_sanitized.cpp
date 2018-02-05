////////////////////////////////////////////////////////////////////////////////
/// @file steeringCurvatureLUTGenerator.cpp
/// @author Venkat Rajagopalan <vrajagop@nrec.ri.cmu.edu> 
/// @date 10/26/2017
/// @brief  
/// @version 0.1
///
/// @details
///
///
/// @par Contract Number(s)
/// 
///
/// @par Government Rights
/// 
///
/// @par Sponsor Rights
/// 
/// 
////////////////////////////////////////////////////////////////////////////////

#include <vehicleModel/libVehicleModel.h>
#include <ais/config/ConfigParser.h>
#include <ais/util/SystemPathModel.h>

#include "MPTG/SteeringRollout.h"
#include "MPTG/SteeringTrajectoryGenerator.h"

#include <iostream>
#include <fstream>
#include <memory>

using namespace vehicleModel;
using namespace MPTG;

////////////////////////////////////////////////////////////////////////////////
int main(int argc, const char* argv[])
{
  //
  std::cout.setf(std::ios_base::fixed);
  std::cout.precision(5);

  ConfigParser configParser;
  configParser.setPath(ais::SystemPathModel::getConfigPath());
  if(!configParser.parseFile("robotsAll.rb"))
    {
      std::cerr << "Could not load robotsAll.rb" <<std::endl;
      return 1;
    }

  ConfigSection section;
  if(!configParser.getSection("Robots",section))
    {
      std::cerr << "BUG BUG BUG NO ROBOT DEFS" << std::endl;
      return 1;
    }

  ConfigSection raptor;
  section.get("raptor",raptor);

  ConfigSection vmconfig;
  raptor.get("vehicleModel",vmconfig);

  // pointer to the vehicle model
  std::shared_ptr<VehicleModel> veh(VehicleModel::createFromConfig(vmconfig));

  // minimum, maximum and delta initial curvature (rad/m)
  const auto max_kappa(veh->getModelParameters().maxCurvatureCommand);
  const auto min_kappa(veh->getModelParameters().minCurvatureCommand);
  const auto delta_kappa = (max_kappa-min_kappa)/10;
  const size_t kappa_range = std::ceil(1+(max_kappa - min_kappa)/delta_kappa);

  // minimum, maximum and delta look-ahead times (s)
  const auto min_la_time(2.0), max_la_time(5.0), delta_la_time(1.0);
  size_t la_time_range = std::ceil(1+(max_la_time-min_la_time)/delta_la_time);

  // minimum, maximum and delta lateral offset ranges (m)
  const auto max_y(10.0), min_y(-10.0), delta_y(0.5);
  const size_t y_range = std::ceil(1+(max_y - min_y)/delta_y);

  // minimum, maximum and delta velocity (m/s)
  const auto min_v(1.0), max_v(10.0), delta_v(0.25);
  size_t v_range = std::ceil(1+(max_v-min_v)/delta_v);
  v_range = 1;

  // minimum, maximum and delta terminal heading (rad)
  const auto min_t(-M_PI/2), max_t(M_PI/2), delta_t(M_PI/36);
  const size_t t_range = std::ceil(1+(max_t-min_t)/delta_t);

  // theoretical max number of trajectories
  const auto max_trajectories = 
    v_range * la_time_range * kappa_range * y_range * t_range;
  
  std::cout << "Max trajectories: " << max_trajectories << "\n";

  // output file
  std::ofstream ofp("SteeringRollout.csv");

  // desired velocity (m/s)
  const auto v(5.0);

  // valid cost 
  const auto good_cost(0.01);

  // valid path length (m)
  const auto max_path_length(30.0);

  //
  size_t search_idx(0);
  size_t good_solution(0);

#pragma omp parallel num_threads(8)
  {

    // pointer to the vehicle model
    std::shared_ptr<VehicleModel> vm(VehicleModel::createFromConfig(vmconfig));

    std::shared_ptr<SteeringRollout> sro(new SteeringRollout(vm));

    SteeringTrajectoryGenerator trajectory_generator(sro.get());

    // initial conditions
    nrec::geometry::ISOPose3D_d initial_pose;
    std::vector<nrec::geometry::ISOPose3D_d> sim_poses;

    std::vector<double> la_times(la_time_range);
    std::size_t timeidx(0);

    for(timeidx = 0; timeidx < la_times.size(); ++timeidx)
      {
    	la_times[timeidx] = min_la_time + timeidx * delta_la_time;
      }
    
    // iterate over look-ahead times
#pragma omp for
    for(timeidx = 0; timeidx < la_times.size(); ++timeidx)
      {
	// desired pose
	ISOPose3D_d desired_pose, temp_desired;

	temp_desired.x() = desired_pose.x() = v * la_times[timeidx];

	// iterate over terminal heading angles
	for(auto t = min_t; t < max_t+0.1; t+= delta_t)
	  {
	    desired_pose.yaw().setRadians(t);

	    // iterate over initial curvatures
	    for(auto k0 = min_kappa; k0 < max_kappa+0.01; k0+= delta_kappa)
	      {
		// lateral offsets to consider
		for(auto y = min_y; y < max_y+0.1; y+= delta_y)
		  {
		    // the parameters
		    std::vector<double> p(SteeringRolloutEvaluator::_num_variable_parameters +
					  SteeringRolloutEvaluator::_num_fixed_parameters,
					  0.0);
		    
		    // initial curvature
		    p[0] = k0;
		    
		    // heuristic path length
		    p[p.size()-1] = 
		      std::max(std::max(desired_pose.x(), y), std::hypot(desired_pose.x(), y));

		    // update the x-coordinate

		    // update desired_pose.y()
		    temp_desired.y() = desired_pose.y() = y;

		    // iterate through the number of batches
		    const auto num_batches(50);
		    double scale(0.0), final_cost(0.0);
		    for(auto k=1; k <= num_batches; ++k)
		      {
			scale = 
			  static_cast<double>(k)/static_cast<double>(num_batches);
			
			// update temp_desired
			temp_desired.y() = scale * desired_pose.y();
			temp_desired.yaw().setRadians(scale*desired_pose.yaw().getRadians());
	  
			// std::cout << "\rCompleted [" << k << "/" << num_batches
			// 		<< "] steps!" << std::flush;
	      
			// solve for parameters
			if(!trajectory_generator.generateTrajectory(initial_pose, temp_desired, v, p, final_cost))
			  {
			    // std::cerr << "Error generating trajectory to desired terminal pose!"
			    // 	      << std::endl;
			    continue;
			    // return -1;
			  } 
		      }
		    // std::cout << std::endl;

		    // tick over good solution counter
		    ++search_idx;

		    if(final_cost <= good_cost && (p.back() <= max_path_length)) 
		      {
#pragma omp critical
			++good_solution;
			std::cout << "\rSearched: " << search_idx 
				  << " Good: " << good_solution
				  << " Max: " << max_trajectories
				  << std::flush;
		      }
		    else
		      {
			{
#pragma omp critical
			  std::cout << "\rSearched: " << search_idx 
				    << " Good: " << good_solution
				    << " Max: " << max_trajectories
				    << std::flush;
			}			
			continue;
		      }
		    
		  
		    // rollout with the parameters
		    if(sro->rollout(p, v, initial_pose, sim_poses))
		      {
			{
#pragma omp critical
			  // dump the simulation results
			  for(auto idx = 0u; idx < sim_poses.size(); ++idx)
			    {
			      ofp << sim_poses[idx].x() << ", " 
				  << sim_poses[idx].y() << ", " 
				  << sim_poses[idx].yaw().getRadians() ;
			      
			      if(idx != sim_poses.size()-1)
				ofp << ",";
			    }
			  ofp << "\n";
			}
			
			// std::cout << "Des: " << desired_pose << "Act: " << sim_poses.back() << "\n";
		      }
		  }
	      }
	  }  
      }
    
  } // OMP PARALLEL

  //
  std::cout << "\n";
  
  // close the stream
  ofp.close();
  
  return 0;
}

