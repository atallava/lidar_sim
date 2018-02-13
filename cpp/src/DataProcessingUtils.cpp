#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <cstring>
#include <fstream>
#include <string>
#include <regex>
#include <ctime>

// hacking boost filesystem bug
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

#include <boost/regex.hpp>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/VizUtils.h>

namespace lidar_sim {
    std::string exec(const char* cmd) {
	char buffer[128];
	std::string result = "";
	std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
	if (!pipe) throw std::runtime_error("popen() failed!");
	while (!feof(pipe.get())) {
	    if (fgets(buffer, 128, pipe.get()) != NULL)
		result += buffer;
	}
	return result;
    }

    int getNumLinesInFile(std::string rel_path_file) {
	std::ostringstream ss;
	ss << "wc -l " << rel_path_file;
	std::string cmd = ss.str();
	std::string cmd_res = exec(cmd.c_str());
	char* buf = strdup(cmd_res.c_str());
	char* token = std::strtok(buf," ");
	int num_lines = atoi(token);

	return num_lines;
    }

    std::string genPCDHeader(int num_pts) {
	std::ostringstream header_ss;
	header_ss <<
	    "# .PCD v.7 - Point Cloud Data file format" << std::endl <<
	    "VERSION .7" << std::endl <<
	    "FIELDS x y z " << std::endl <<
	    "SIZE 4 4 4" << std::endl <<
	    "TYPE F F F" << std::endl <<
	    "COUNT 1 1 1" << std::endl <<
	    "WIDTH " << num_pts << std::endl <<
	    "HEIGHT 1" << std::endl <<
	    "VIEWPOINT 0 0 0 1 0 0 0" << std::endl <<
	    "POINTS " << num_pts << std::endl <<
	    "DATA ascii" << std::endl;
	return header_ss.str();
    }

    std::string getPtsLineFromSectionLine(std::string line)
    {
	std::istringstream iss(line);

	double data;
	// packet id
	iss >> data; 
	// packet timestamp
	iss >> data;
	iss >> data;

	double x, y, z;
	iss >> x;
	iss >> y;
	iss >> z;
	std::string line_pts = std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
	
	return line_pts;
    }

    void subsampleFile(std::string rel_path_file, std::string rel_path_file_subsampled, int subsample_factor)
    {
	// open input file
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_file;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	// open output file
	std::ofstream subsampled_file(rel_path_file_subsampled);
	std::cout << "Writing to: " << rel_path_file_subsampled << std::endl;

	std::cout << "Subsampling factor: " << subsample_factor << std::endl;

	// subsample
	std::string current_line;
	int count = 0;
	while(std::getline(file,current_line))
	{
	    if (count % subsample_factor == 0)
		subsampled_file << current_line << std::endl;

	    count++;
	}

	// close files
	file.close();
	subsampled_file.close();
    }    

    void sectionOfSection(std::string rel_path_input, std::string rel_path_output, double start_time, double end_time)
    {
	// open input file
	std::ifstream input_file(rel_path_input);
	std::cout << "Reading from: " << rel_path_input << std::endl;
	if (!input_file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::ofstream output_file(rel_path_output);
	std::cout << "Writing to: " << rel_path_output << std::endl;

	std::string current_line;
	while(std::getline(input_file,current_line))
	{
	    double packet_timestamp_sec;
	    double packet_timestamp_nanosec;
	    double packet_timestamp;
	    int packet_id;
	    std::istringstream iss(current_line);

	    // packet id
	    iss >> packet_id; 

	    // packet timestamp
	    iss >> packet_timestamp_sec;
	    iss >> packet_timestamp_nanosec;
	    packet_timestamp = packet_timestamp_sec + packet_timestamp_nanosec*1e-9;

	    if ((packet_timestamp > start_time) && (packet_timestamp < end_time))
	    {
		output_file << current_line;
		output_file << std::endl;
	    }

	    if (packet_timestamp > end_time)
		break;
	}

	// close files
	input_file.close();
	output_file.close();
    }

    void prependPCDHeaderToFile(std::string rel_path_input, std::string rel_path_output)
    {
	std::ifstream input_file(rel_path_input);
	std::cout << "Reading from: " << rel_path_input << std::endl;
	if (!input_file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::ofstream output_file(rel_path_output);
	std::cout << "Writing to: " << rel_path_output << std::endl;

	int num_pts = getNumLinesInFile(rel_path_input); 
	std::string pcd_header = genPCDHeader(num_pts);
	
	// write header
	output_file << pcd_header;

	// write rest of input to output
	std::string current_line;
	while(std::getline(input_file,current_line))
	    output_file << current_line << std::endl;

	// close files
	input_file.close();
	output_file.close();
    }

    std::string genDetailLine(double packet_timestamp, std::vector<double> imu_pose, Eigen::Matrix<float,4,1> pt)
    {
	std::ostringstream ss;
	ss << packet_timestamp << " ";
	for (size_t i = 0; i < 6; ++i)
	    ss << imu_pose[i] << " ";
	for (size_t i = 0; i < 3; ++i)
	    ss << pt[i] << " ";
	ss << std::endl;

	return ss.str();
    }

    std::vector<std::vector<double> > loadPtsFromXYZFile(std::string rel_path_input, int verbose)
    {
	std::ifstream input_file(rel_path_input);
	if (!input_file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_input;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	if (verbose)
	    std::cout << "Reading pts from: " << rel_path_input << std::endl;

	std::vector<std::vector<double> > pts;
	
	std::string current_line;
	while(std::getline(input_file,current_line))
	{
	    std::istringstream iss(current_line);
	    double x, y, z;
	    iss >> x;
	    iss >> y;
	    iss >> z;
	    
	    std::vector<double> this_pt;
	    this_pt.push_back(x);
	    this_pt.push_back(y);
	    this_pt.push_back(z);

	    pts.push_back(this_pt);
  	}

	return pts;
    }

    std::tuple<std::vector<std::string>, std::vector<std::vector<double> > >
    loadAnnotations(const std::string rel_path_file)
    {
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_file;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::vector<std::string> object_classes;
	std::vector<std::vector<double> > object_xy_posns;

	std::string current_line;
	while(std::getline(file,current_line))
	{
	    std::istringstream iss(current_line);

	    std::string object_class;
	    iss >> object_class;
	    object_classes.push_back(object_class);

	    std::vector<double> xy(2, 0);
	    iss >> xy[0]; iss >> xy[1];
	    object_xy_posns.push_back(xy);
	}

	file.close();

	return std::make_tuple(object_classes, object_xy_posns);
    }

    void writeStringToFile(const std::string str, const std::string rel_path_output, int verbose)
    {
	std::ofstream file(rel_path_output);
	if (verbose)
	    std::cout << "Writing string to: " << rel_path_output << std::endl;

	file << str;
	file.close();
    }

    void writeQueriedBlocks(const std::string rel_path_output, const std::vector<int> &triangle_block_ids, 
			    const std::vector<int> &ellipsoid_block_ids)
    {
	std::ofstream file(rel_path_output);
	std::cout << "Writing queried blocks to: " << rel_path_output << std::endl;

	// write triangle block ids
	std::ostringstream ss;
	for(size_t i = 0; i < triangle_block_ids.size(); ++i)
	    ss << triangle_block_ids[i] << " ";
	ss << std::endl;

	file << ss.str();

	// write ellipsoid block ids
	ss.str("");
	ss.clear();
	for(size_t i = 0; i < ellipsoid_block_ids.size(); ++i)
	    ss << ellipsoid_block_ids[i] << " ";
	ss << std::endl;

	file << ss.str();
    }

    std::tuple<std::vector<int>, std::vector<int> >
    readQueriedBlocks(const std::string rel_path_file)
    {
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_file;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}
	
	// read triangle block ids
	std::vector<int> triangle_block_ids;
	std::string current_line;
	std::getline(file, current_line);
	std::istringstream iss(current_line);
	int id;
	while(iss >> id)
	    triangle_block_ids.push_back(id);

	// read ellipsoid block ids
	std::vector<int> ellipsoid_block_ids;
	std::getline(file, current_line);
	iss.clear();
	iss.str(current_line);
	while(iss >> id)
	    ellipsoid_block_ids.push_back(id);

	return std::make_tuple(triangle_block_ids, ellipsoid_block_ids);
    }

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double> >
    getVecsFromPts(const std::vector<std::vector<double> > &pts)
    {
	std::vector<double> x(pts.size(), 0);
	std::vector<double> y(pts.size(), 0);
	std::vector<double> z(pts.size(), 0);

	for(size_t i = 0; i < pts.size(); ++i)
	{
	    x[i] = pts[i][0];
	    y[i] = pts[i][1];
	    z[i] = pts[i][2];
	}

	return std::make_tuple(x, y, z);
    }

    std::vector<std::vector<double> > loadArray(std::string rel_path_file, int n_cols)
    {
	// open input file
	std::ifstream file(rel_path_file);
	std::cout << "Reading from: " << rel_path_file << std::endl;
	if (!file)
	{
	    std::stringstream ss_err_msg;
	    ss_err_msg << "failed to open file " << rel_path_file;
	    throw std::runtime_error(ss_err_msg.str().c_str());
	}

	std::vector<std::vector<double> > array;

	std::string current_line;
	while(std::getline(file,current_line))
	{
	    std::istringstream iss(current_line);
	    
	    std::vector<double> vec(n_cols, 0);
	    for(size_t i = 0; i < (size_t)n_cols; ++i)
		iss >> vec[i];

	    array.push_back(vec);
	}

	file.close();

	return array;
    }
    
    alglib::real_2d_array convertStlPtsToAlglibPts(const std::vector<std::vector<double> > &pts)
    {
	alglib::real_2d_array pts_alglib;
	pts_alglib.setlength(pts.size(), 3);

	for(size_t i = 0; i < pts.size(); ++i)
	    for(size_t j = 0; j < 3; ++j)
		pts_alglib[i][j] = pts[i][j];

	return pts_alglib;
    }

    std::vector<int> convertAlglib1DIntArrayToStlVector(const alglib::integer_1d_array &vec)
    {
	std::vector<int> vec_stl;
	
	for(size_t i = 0; i < (size_t)vec.length(); ++i)
	    vec_stl.push_back((int)vec[i]);

	return vec_stl;
    }

    Eigen::MatrixXd stlVecToEigen(const std::vector<double> &vec)
    {
	Eigen::MatrixXd vec_eigen(vec.size(), 1);
	for(size_t i = 0; i < vec.size(); ++i)
	    vec_eigen(i) = vec[i];

	return vec_eigen;
    }

    Eigen::MatrixXd stlArrayToEigen(const std::vector<std::vector<double> > &array)
    {
	Eigen::MatrixXd array_eigen(array.size(), array[0].size());
	for(size_t i = 0; i < array.size(); ++i)
	    for(size_t j = 0; j < array[0].size(); ++j)
		array_eigen(i,j) = array[i][j];

	return array_eigen;
    }

    std::vector<std::vector<double> > EigenToStlArray(const Eigen::MatrixXd &array)
    {
	std::vector<std::vector<double> > array_stl(array.rows(), 
						    std::vector<double>(array.cols()));
	for(size_t i = 0; i < (size_t)array.rows(); ++i)
	    for(size_t j = 0; j < (size_t)array.cols(); ++j)
		array_stl[i][j] = array(i,j);

	return array_stl;
    }

    std::vector<std::vector<int> > doubleToIntArray(const std::vector<std::vector<double> > &array)
    {
	std::vector<std::vector<int> > int_array(array.size(), std::vector<int>(array[0].size()));
	for(size_t i = 0; i < array.size(); ++i)
	    for(size_t j = 0; j < array[0].size(); ++j)
		int_array[i][j] = array[i][j];

	return int_array;
    }

    flann::Matrix<double> stlArrayToFlannMatrix(const std::vector<std::vector<double> > &array)
    {
	std::vector<double> array_unrolled;
	// unrolling in row-major order
	for (size_t i = 0; i < array.size(); ++i)
	    array_unrolled.insert(array_unrolled.end(), array[i].begin(), array[i].end());

	flann::Matrix<double> mat(array_unrolled.data(), array.size(), array[0].size());

	return mat;
    }

    std::vector<double> convertIntVecToDoubleVec(std::vector<int> vec_in)
    {
	std::vector<double> vec_out;
	for(size_t i = 0; i < vec_in.size(); ++i)
	    vec_out.push_back(vec_in[i]);

	return vec_out;
    }

    std::vector<std::string> getPatternMatchingFiles(std::string rel_path_dir, boost::regex pattern)
    {
	std::vector< std::string > matching_filenames;

	boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
	for( boost::filesystem::directory_iterator i( rel_path_dir ); i != end_itr; ++i )
	{
	    if( !boost::filesystem::is_regular_file( i->status() ) ) continue;

	    std::string filename = i->path().filename().string();

	    if (boost::regex_search(filename, pattern))
		matching_filenames.push_back(filename);
	}

	return matching_filenames;
    }

    std::tuple<std::vector<std::string>, std::vector<std::vector<std::string> > >
    getPatternMatchingFiles(std::string rel_path_dir, boost::regex pattern, int num_captures)
    {
	std::vector< std::string > matching_filenames;
	std::vector<std::vector<std::string> > captures;

	boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
	for( boost::filesystem::directory_iterator i( rel_path_dir ); i != end_itr; ++i )
	{
	    if( !boost::filesystem::is_regular_file( i->status() ) ) continue;

	    std::string filename = i->path().filename().string();

	    std::vector<std::string> this_match_captures;
	    boost::match_results<std::string::const_iterator> what;
	    if (boost::regex_search(filename, what, pattern))
	    {
		matching_filenames.push_back(filename);

		if ( (int)(what.size()-1) != num_captures )
		{
		    std::stringstream ss_err_msg;
		    ss_err_msg << "number of boost regex captures:" << what.size()-1 << " does not match expected: " << num_captures;
		    throw std::logic_error(ss_err_msg.str().c_str());
		}
		
		// 1 because what[0] is entire string
		for(size_t j = 1; j < (size_t)num_captures+1; ++j)
		    this_match_captures.push_back(what[j]);

		captures.push_back(this_match_captures);
	    }
	}

	return std::make_tuple(matching_filenames, captures);
    }

    std::vector<int> getGroundBlockIds(const std::string rel_path_ground_models_dir, const int section_id)
    {
	std::ostringstream ss;
	ss << "section_" << std::setw(2) << std::setfill('0')
	   << section_id << "_block" << "_([0-9]+)"
	   << "_ground.xyz";
	boost::regex pattern(ss.str());
	int num_captures = 1;

	std::vector<std::string> matching_filenames;
	std::vector<std::vector<std::string> > captures;
	std::tie(matching_filenames, captures) = 
	    getPatternMatchingFiles(rel_path_ground_models_dir, pattern, num_captures);

	std::vector<int> ids;
	for(size_t i = 0; i < captures.size(); ++i)
	    ids.push_back(std::stoi(captures[i][0]));

	// sort ids
	std::sort(ids.begin(), ids.end());

	return ids;
    }

    std::vector<int> getNonGroundBlockIds(const std::string rel_path_non_ground_models_dir, const int section_id)
    {
	std::ostringstream ss;
	ss << "section_" << std::setw(2) << std::setfill('0')
	   << section_id << "_block" << "_([0-9]+)"
	   << "_non_ground.xyz";
	boost::regex pattern(ss.str());
	int num_captures = 1;

	std::vector<std::string> matching_filenames;
	std::vector<std::vector<std::string> > captures;
	std::tie(matching_filenames, captures) = 
	    getPatternMatchingFiles(rel_path_non_ground_models_dir, pattern, num_captures);

	std::vector<int> ids;
	for(size_t i = 0; i < captures.size(); ++i)
	    ids.push_back(std::stoi(captures[i][0]));

	// sort ids
	std::sort(ids.begin(), ids.end());

	return ids;
    }

    std::vector<int> getEllipsoidModelBlockIds(const std::string rel_path_ellipsoid_models_dir, const int section_id)
    {
	std::ostringstream ss;
	ss << "section_" << std::setw(2) << std::setfill('0') 
	   << section_id << "_block" << "_([0-9]+)" 
	   << "_non_ground_ellipsoids.txt";
	boost::regex pattern(ss.str());
	int num_captures = 1; // who checks consistency between num_captures and pattern?

	std::vector<std::string> matching_filenames;
	std::vector<std::vector<std::string> > captures;
	std::tie(matching_filenames, captures) = 
	    getPatternMatchingFiles(rel_path_ellipsoid_models_dir, pattern, num_captures);

	std::vector<int> ids;
	for(size_t i = 0; i < captures.size(); ++i)
	    ids.push_back(std::stoi(captures[i][0]));

	// note: important that ids are sorted!
	std::sort(ids.begin(), ids.end());

	return ids;
    }

    std::vector<int> getTriangleModelBlockIds(const std::string rel_path_triangle_models_dir, const int section_id)
    {
	std::ostringstream ss;
	ss << "section_" << std::setw(2) << std::setfill('0') 
	   << section_id << "_block" << "_([0-9]+)" 
	   << "_ground_triangles.txt";
	boost::regex pattern(ss.str());
	int num_captures = 1; // who checks consistency between num_captures and pattern?

	std::vector<std::string> matching_filenames;
	std::vector<std::vector<std::string> > captures;
	std::tie(matching_filenames, captures) = 
	    getPatternMatchingFiles(rel_path_triangle_models_dir, pattern, num_captures);

	std::vector<int> ids;
	for(size_t i = 0; i < captures.size(); ++i)
	    ids.push_back(std::stoi(captures[i][0]));

	// note: important that ids are sorted!
	std::sort(ids.begin(), ids.end());

	return ids;
    }

    std::vector<int> getObjectMeshIds(const std::string rel_path_object_meshes_dir)
    {
	boost::regex pattern("([0-9]+)");
	std::vector<std::string> matching_filenames;
	std::vector<std::vector<std::string> > captures;
	int num_captures = 1;

	std::tie(matching_filenames, captures) = 
	    getPatternMatchingFiles(rel_path_object_meshes_dir, pattern, num_captures);

	std::vector<int> ids;
	for(size_t i = 0; i < captures.size(); ++i)
	    ids.push_back(std::stoi(captures[i][0]));

	// note: important that ids are sorted!
	std::sort(ids.begin(), ids.end());

	return ids;
    }

    std::string getDateString(const std::string format)
    {
	// copied from SO
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,sizeof(buffer),format.c_str(),timeinfo);
	std::string str(buffer);

	return str;
    }


    std::string genPathSection(const int section_id)
    {
    	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
    	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
    	   << "/section_" << std::setw(2) << std::setfill('0') << section_id 
    	   << "_subsampled.xyz";

    	return ss.str();
    }

    std::string genPathPosesLog()
    {
	std::string rel_path_poses_log = "/usr0/home/atallav1/lidar_sim/data/taylorJune2014/Pose/PoseAndEncoder_1797_0000254902_wgs84_wgs84.fixed";
	return rel_path_poses_log;
    }

    std::string genPathImuPosnNodes(const int section_id)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
	   << "/imu_posn_nodes.txt";

	return ss.str();
    }

    std::string genPathBlockNodeIdsGround(const int section_id)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
	   << "/hg_sim/blocks_info/block_node_ids_ground.txt";

	return ss.str();
    }

    std::string genPathBlockNodeIdsNonGround(const int section_id)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
	   << "/hg_sim/blocks_info/block_node_ids_non_ground.txt";

	return ss.str();
    }

    std::string genPathNonGroundBlockPts(const int section_id, const int block_id)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	   << "/hg_sim/blocks_info/section_" << std::setw(2) << std::setfill('0') << section_id
	   << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_non_ground.xyz";

	return ss.str();
    }

    std::string genPathGroundBlockPts(const int section_id, const int block_id)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	   << "/hg_sim/blocks_info/section_" << std::setw(2) << std::setfill('0') << section_id
	   << "_block_" << std::setw(2) << std::setfill('0') << block_id << "_ground.xyz";

	return ss.str();
    }

    std::string genPathHgModelsDir(const int section_id, const std::string sim_version)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id
	   << "/hg_sim/version_" << sim_version;

	return ss.str();
    }

    std::string genPathRealPtsRef(const int section_id, const std::string sim_type, 
				  const std::string sim_version, const std::string query_type, const int tag)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
	   << "/" << sim_type << "_sim" << "/version_" << sim_version
	   << "/" << query_type << "_real_pts";

	if (tag == -1)
	    ss << ".xyz";
	else
	    ss << "_" << tag << ".xyz";

	return ss.str();
    }

    std::string genPathSimPts(const int section_id, const std::string sim_type, 
			      const std::string sim_version, const std::string query_type, const int tag)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
	   << "/" << sim_type << "_sim" << "/version_" << sim_version
	   << "/" << query_type << "_sim_pts";

	if (tag == -1)
	    ss << ".xyz";
	else
	    ss << "_" << tag << ".xyz";

	return ss.str();
    }

    std::string genPathSimDetail(const int section_id, const std::string sim_type, 
				 const std::string sim_version, const std::string query_type, const int tag)
    {
	std::ostringstream ss;
    	ss << "/usr0/home/atallav1/lidar_sim/cpp"
	   << "/data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
	   << "/" << sim_type << "_sim" << "/version_" << sim_version
	   << "/" << query_type << "_sim_detail";

	if (tag == -1)
	    ss << ".txt";
	else
	    ss << "_" << tag << ".txt";

	return ss.str();
    }

    bool myMkdir(std::string rel_path_dir)
    {
	// if directory exists, do nothing
	// todo: have option of rewriting even if exists?
	if (boost::filesystem::exists(rel_path_dir)) {
	    std::cout << "Directory " << rel_path_dir << " exists" << std::endl;
	    return false;
	}
	else {
	    std::cout << "Creating directory " << rel_path_dir << std::endl;
	    return boost::filesystem::create_directory(rel_path_dir);
	}
    }

    bool myMkdirs(std::string rel_path_dir)
    {
	if (boost::filesystem::exists(rel_path_dir)) {
	    std::cout << "Directory " << rel_path_dir << " exists" << std::endl;
	    return false;
	}
	else {
	    std::cout << "Creating directory " << rel_path_dir << std::endl;
	    return boost::filesystem::create_directories(rel_path_dir);
	}
    }
}
