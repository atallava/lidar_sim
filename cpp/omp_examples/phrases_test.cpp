// test some phrases of code that appear in the sim 

#include <iostream>
#include <vector>
#include <omp.h>

#include <lidar_sim/AlgoStateEstUtils.h>
#include <lidar_sim/SectionLoader.h>

using namespace lidar_sim;

int main() 
{
    typedef std::vector<std::vector<double> > Pts;

    clock_t start_time = clock();

    // load real packets
    int section_scans_id = 4;
    std::string scans_version = "260118"; 
    std::string rel_path_real_packets = 
	algo_state_est::genRelPathPacketsToProcess(section_scans_id, scans_version, "real");
    SectionLoader real_packets(rel_path_real_packets);

    size_t n_packets = real_packets.m_packet_ids.size();
    n_packets = 10; // todo: change/ delete me! limiting for debug
    std::vector<size_t> n_real_pts_per_packet(n_packets, 0);
    std::vector<Pts> real_pts_per_packet;
    real_pts_per_packet.resize(n_packets);
    
    for (size_t i = 0; i < n_packets; ++i)
    {
	double t = real_packets.m_packet_timestamps[i];

	// packet pts
	Pts pts = real_packets.getPtsAtTime(t);
	n_real_pts_per_packet[i] = pts.size();
    	real_pts_per_packet[i].resize(n_real_pts_per_packet[i], std::vector<double>(3));
    }

    int num_threads = 1;
#pragma omp parallel num_threads (num_threads)
    {
#pragma omp for
	for(size_t i = 0; i < n_packets; ++i)
	{	
	    double t = real_packets.m_packet_timestamps[i];

	    // packet pts
	    Pts pts = real_packets.getPtsAtTime(t);
	    std::cout << "packet indx: " << i << " n pts: " << pts.size() << std::endl;
	    real_pts_per_packet[i] = pts;
	}
    }

    std::cout << real_pts_per_packet[1].size() << std::endl;

    double elapsed_time = (clock() - start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s. " << std::endl;

    return(1);
}
