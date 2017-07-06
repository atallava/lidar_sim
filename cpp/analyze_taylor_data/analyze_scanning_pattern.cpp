#include <iostream>
#include <iomanip>
#include <tuple>
#include <ctime>

#include <lidar_sim/SectionLoader.h>
#include <lidar_sim/RayDirnServer.h>
#include <lidar_sim/PoseServer.h>
#include <lidar_sim/ModelingUtils.h>
#include <lidar_sim/LaserCalibParams.h>
#include <lidar_sim/LaserUtils.h>
#include <lidar_sim/VizUtils.h>

using namespace lidar_sim;

std::string genRelPathSection(int section_id)
{
    std::ostringstream ss;
    ss << "data/sections/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "/section_" << std::setw(2) << std::setfill('0') << section_id 
       << "_world_frame_subsampled.xyz";

    return ss.str();
}

int main(int argc, char **argv)
{
    clock_t start_time = clock();

    std::vector<double> ray_origin{0, 0, 0};
    std::vector<std::vector<double> > packet_pts{
	{1.472,1.7199,-1.3426},
	{1.7568,2.0526,-0.4439},
	{1.5011,1.7539,-1.2971},
	{1.6137,1.8853,-0.3488},
	{1.6374,1.9131,-1.3389},
	{1.8582,2.1711,-1.4354},
	{2.4438,2.8552,-1.7789},
	{10.2981,12.032,-1.1075},
	{2.4415,2.8525,-1.6717},
	{8.344,9.7488,-0.5984},
	{3.5628,4.1627,-2.2886},
	{7.5649,8.8386,0},
	{7.8307,9.1492,0.2796},
	{1.699,1.9851,-0.8829},
	{7.3801,8.6226,0.5293},
	{1.676,1.9581,-0.8043},
	{8.3846,9.7963,0.9017},
	{8.5124,9.9457,1.2213},
	{1.6657,1.9462,-0.6706},
	{1.6906,1.9753,-0.6161},
	{6.4585,7.5459,1.3959},
	{1.655,1.9336,-0.541},
	{6.6898,7.8161,1.6903},
	{1.7598,2.0561,-0.5099},
	{6.4258,7.5077,1.8619},
	{1.4937,1.7353,-1.3579},
	{1.7432,2.0253,-0.439},
	{1.5982,1.8567,-1.3764},
	{1.6228,1.8853,-0.3496},
	{1.7246,2.0036,-1.4056},
	{1.8881,2.1936,-0.3385},
	{8.1217,9.4356,-0.8706},
	{8.4728,9.8436,-0.6057},
	{7.4558,8.6621,-0.2653},
	{8.3059,9.6497,0},
	{7.8928,9.1697,0.2809},
	{1.7144,1.9918,-0.888},
	{7.3819,8.5762,0.5277},
	{1.6478,1.9144,-0.7882},
	{6.5377,7.5954,0.7008},
	{1.786,2.0749,-0.785},
	{1.6484,1.9151,-0.6615},
	{6.5495,7.6091,1.1741},
	{1.7152,1.9927,-0.623},
	{6.5441,7.6029,1.4098},
	{1.6667,1.9364,-0.5431},
	{6.8648,7.9754,1.7289},
	{1.8014,2.0929,-0.5203},
	{6.4198,7.4585,1.8541},
	{1.4974,1.7299,-1.3568},
	{1.7657,2.0398,-0.4432},
	{1.6969,1.9603,-1.4568},
	{1.6358,1.8898,-0.3513},
	{1.7486,2.0201,-1.4206},
	{1.8786,2.1703,-0.3357},
	{1.8762,2.1675,-1.4399},
	{7.8146,9.0277,-1.114},
	{2.455,2.8361,-1.7755},
	{2.4609,2.843,-1.6741},
	{8.1211,9.3819,-0.5787},
	{7.4564,8.614,-0.2645},
	{7.5631,8.7373,0},
	{3.7983,4.388,-2.1123},
	{7.8503,9.069,0.2785},
	{1.7559,2.0285,-0.9066},
	{8.6676,10.0133,0.6176},
	{1.6519,1.9083,-0.7876},
	{6.5602,7.5786,0.7009},
	{1.6974,1.9609,-0.7437},
	{8.3359,9.63,1.1883},
	{1.6639,1.9222,-0.6655},
	{6.6032,7.6283,1.1799},
	{1.6902,1.9526,-0.6119},
	{1.7298,1.9983,-0.5618},
	{6.8986,7.9696,1.7318},
	{1.8009,2.0804,-0.5184},
	{6.4187,7.4152,1.8478},
	{1.5116,1.7358,-1.365},
	{1.7912,2.0569,-0.4481},
	{1.7451,2.004,-1.493},
	{1.6856,1.9357,-0.3607},
	{1.772,2.0349,-1.4347},
	{1.8916,2.1722,-0.3368},
	{1.811,2.0797,-1.3852},
	{7.8321,8.994,-1.1127},
	{2.4812,2.8492,-1.7883},
	{10.3391,11.8728,-1.1009},
	{2.4694,2.8357,-1.6741},
	{8.1607,9.3713,-0.5795},
	{7.4793,8.5888,-0.2644},
	{3.8014,4.3653,-2.1068},
	{7.7774,8.9311,0.275},
	{1.7595,2.0205,-0.9053},
	{7.4509,8.5562,0.5291},
	{1.6663,1.9135,-0.7918},
	{1.7044,1.9573,-0.7442},
	{1.6823,1.9319,-0.6706},
	{6.6519,7.6386,1.1845},
	{1.6653,1.9123,-0.6008},
	{6.5254,7.4934,1.3965},
	{1.7729,2.0359,-0.5738},
	{7.0156,8.0563,1.7551},
	{1.7502,2.0099,-0.5021},
	{6.482,7.4436,1.8597},
	{1.511,1.7248,-1.3599},
	{1.8207,2.0783,-0.4539},
	{1.7062,1.9476,-1.4548},
	{1.7083,1.9501,-0.3644},
	{1.9014,2.1704,-1.5342},
	{1.9098,2.18,-0.3389},
	{1.7866,2.0393,-1.3618},
	{7.8457,8.9557,-1.1108},
	{2.4968,2.85,-1.7935},
	{8.2221,9.3854,-0.8725},
	{2.4814,2.8325,-1.6766},
	{8.1885,9.347,-0.5795},
	{7.5206,8.5846,-0.265},
	{7.8692,8.9826,0},
	{1.7536,2.0017,-0.9686},
	{7.8987,9.0163,0.2783},
	{1.7405,1.9867,-0.8925},
	{7.4631,8.519,0.5282},
	{1.6707,1.9071,-0.7912},
	{7.0612,8.0603,0.7493},
	{1.6938,1.9334,-0.7371},
	{8.4322,9.6253,1.1938},
	{1.7722,2.0229,-0.704},
	{6.6352,7.574,1.1775},
	{1.6697,1.9059,-0.6004},
	{6.5867,7.5187,1.4048},
	{1.739,1.985,-0.5609},
	{6.5765,7.507,1.6397},
	{1.7225,1.9662,-0.4925},
	{6.5313,7.4554,1.8674},
	{1.5036,1.7061,-1.3487},
	{1.866,2.1173,-0.4637},
	{1.6809,1.9072,-1.4284},
	{1.739,1.9732,-0.3696},
	{1.8623,2.1131,-1.4976},
	{1.9215,2.1803,-0.3399},
	{1.8659,2.1171,-1.4175},
	{2.504,2.8412,-1.7926},
	{2.491,2.8264,-1.6774},
	{7.5526,8.5698,-0.2652},
	{3.2827,3.7248,-1.9387},
	{7.7596,8.8047,0},
	{1.7595,1.9965,-0.9686},
	{7.8091,8.8608,0.2742},
	{1.7301,1.9631,-0.8842},
	{7.458,8.4624,0.526},
	{1.6764,1.9021,-0.7912},
	{7.064,8.0154,0.7471},
	{7.1086,8.0659,1.003},
	{1.7206,1.9523,-0.6812},
	{6.5881,7.4753,1.1652},
	{1.6496,1.8718,-0.5912},
	{6.6784,7.5778,1.4196},
	{1.7358,1.9696,-0.558},
	{6.5648,7.4489,1.6313},
	{1.7062,1.936,-0.4862},
	{6.5131,7.3902,1.856},
	{1.499,1.6919,-1.3405},
	{1.8623,2.102,-0.4614},
	{1.5737,1.7762,-1.3333},
	{1.7297,1.9523,-0.3666},
	{1.7636,1.9906,-1.414},
	{1.9364,2.1857,-0.3415},
	{1.875,2.1163,-1.4202},
	{1.9214,2.1687,-0.2703},
	{2.8063,3.1675,-2.0031},
	{8.4239,9.5081,-0.8883},
	{8.2034,9.2593,-0.5769},
	{7.6042,8.5829,-0.2662},
	{3.4049,3.8431,-2.0049},
	{7.9842,9.0119,0},
	{1.7673,1.9947,-0.97},
	{7.9065,8.9241,0.2768},
	{1.7453,1.9699,-0.8893},
	{8.4909,9.5838,0.5971},
	{1.6826,1.8992,-0.7918},
	{9.01,10.1696,0.9501},
	{1.7199,1.9412,-0.7437},
	{1.7154,1.9362,-0.6772},
	{6.6142,7.4655,1.1664},
	{1.6506,1.8631,-0.5898},
	{6.6326,7.4862,1.4056},
	{1.6437,1.8552,-0.5268},
	{6.6013,7.4509,1.6355},
	{1.6891,1.9065,-0.4799},
	{7.695,8.6854,2.1863},
	{1.4877,1.6697,-1.3262},
	{1.9036,2.1366,-0.4701},
	{1.7312,1.9431,-0.3657},
	{1.5037,1.6877,-1.2018},
	{1.9795,2.2218,-0.348},
	{1.8832,2.1137,-1.422},
	{1.9222,2.1574,-0.2696},
	{7.9487,8.9214,-0.8355},
	{2.5014,2.8075,-1.6741},
	{7.6428,8.5781,-0.2667},
	{3.4069,3.8238,-1.9998},
	{3.4569,3.8799,-1.8914},
	{7.9088,8.8766,0.276},
	{1.7697,1.9862,-0.8989},
	{8.3675,9.3915,0.5866},
	{1.7095,1.9187,-0.8019},
	{6.6374,7.4497,0.6977},
	{1.7649,1.9809,-0.7608},
	{1.7311,1.943,-0.6812},
	{6.6112,7.4203,1.1622},
	{1.6623,1.8657,-0.5921},
	{6.6231,7.4336,1.3992},
	{1.615,1.8127,-0.516},
	{6.5945,7.4015,1.6287},
	{1.6696,1.8739,-0.4729},
	{6.536,7.3358,1.8512},
	{1.4929,1.665,-1.3262},
	{1.8998,2.1189,-0.4676},
	{1.7006,1.8967,-1.4313},
	{1.7373,1.9376,-0.3657},
	{1.5172,1.6921,-1.2084},
	{1.9878,2.217,-0.3482},
	{1.8612,2.0758,-1.4004},
	{1.9249,2.1469,-0.269},
	{7.8888,8.7984,-0.8263},
	{8.2769,9.2313,-0.5782},
	{3.3757,3.7649,-2.1121},
	{7.6857,8.5719,-0.2673},
	{3.4152,3.8089,-1.9977},
	{7.9187,8.8318,0},
	{1.8037,2.0117,-0.913},
	{1.857,2.0711,-0.868},
	{6.5516,7.307,0.6863},
	{1.7852,1.9911,-0.7668},
	{7.1839,8.0122,1.004},
	{1.7166,1.9145,-0.6731},
	{6.4754,7.222,1.1343},
	{1.6876,1.8822,-0.599},
	{6.6041,7.3656,1.3903},
	{1.6351,1.8236,-0.5206},
	{6.5584,7.3146,1.6141},
	{1.6886,1.8833,-0.4766},
	{6.5249,7.2772,1.8415},
	{1.501,1.6647,-1.3293},
	{1.9084,2.1165,-0.4682},
	{1.7643,1.9567,-1.4803},
	{1.5148,1.68,-1.2028},
	{2.0047,2.2233,-0.3501},
	{1.8251,2.0242,-1.369},
	{1.9323,2.143,-0.2692},
	{7.9254,8.7897,-0.8276},
	{3.0771,3.4127,-2.0459},
	{8.3789,9.2927,-0.5835},
	{7.6976,8.537,-0.2669},
	{2.9118,3.2293,-1.6979},
	{8.151,9.0399,0},
	{8.4982,9.425,0.2946},
	{1.8043,2.001,-0.9104},
	{7.1655,7.9469,0.499},
	{1.8525,2.0546,-0.8632},
	{6.5519,7.2664,0.6842},
	{1.7638,1.9561,-0.7552},
	{1.6869,1.8709,-0.6595},
	{6.4809,7.1877,1.1318},
	{1.6785,1.8616,-0.5939},
	{6.5504,7.2648,1.3748},
	{1.6559,1.8365,-0.5256},
	{6.5775,7.2948,1.6137},
	{1.6175,1.7939,-0.4551},
	{6.6373,7.3611,1.8674},
	{1.521,1.6768,-1.3426},
	{1.9332,2.1313,-0.4727},
	{1.7548,1.9346,-1.4676},
	{1.5091,1.6637,-1.1943},
	{7.9836,8.8016,-1.3896},
	{1.8251,2.0121,-1.3645},
	{1.9359,2.1343,-0.2688},
	{2.842,3.1332,-2.0022},
	{7.9688,8.7853,-0.8294},
	{3.1965,3.524,-2.1183},
	{8.2615,9.108,-0.5734},
	{3.3378,3.6798,-2.0751},
	{7.8182,8.6193,-0.2702},
	{1.8064,1.9915,-0.9085},
	{7.0696,7.7939,0.4907},
	{1.7842,1.9671,-0.8287},
	{6.6445,7.3253,0.6916},
	{1.7256,1.9024,-0.7365},
	{1.6652,1.8358,-0.6488},
	{6.5489,7.2199,1.1399},
	{1.6788,1.8508,-0.5921},
	{6.5719,7.2453,1.3748},
	{1.6587,1.8286,-0.5248},
	{6.6058,7.2826,1.6154},
	{1.5991,1.7629,-0.4484},
	{6.6353,7.3152,1.8608},
	{1.5282,1.6748,-1.3446},
	{1.9408,2.127,-0.4731},
	{1.7077,1.8715,-1.4235},
	{1.5271,1.6736,-1.2047},
	{1.8275,2.0027,-1.3618},
	{2.8866,3.1634,-2.0271},
	{7.9962,8.7631,-0.8295},
	{3.2513,3.5631,-2.1476},
	{8.2791,9.0731,-0.5728},
	{3.1745,3.479,-1.9672},
	{7.9314,8.692,-0.2732},
	{1.8646,2.0435,-0.9347},
	{6.6968,7.339,0.4633},
	{1.8145,1.9885,-0.84},
	{6.7039,7.3468,0.6955},
	{1.7144,1.8789,-0.7293},
	{1.6628,1.8223,-0.6458},
	{6.5716,7.2019,1.1401},
	{1.6659,1.8257,-0.5856},
	{6.5827,7.2141,1.3725},
	{1.8078,1.9812,-0.5701},
	{6.6114,7.2455,1.6115},
	{1.5924,1.7451,-0.4451},
	{6.6994,7.3419,1.8726}};
    
    RayDirnServer ray_dirn_server;

    std::vector<double> unrolled_yaws;
    std::vector<double> unrolled_pitches;
    std::vector<std::vector<double> > unrolled_pts;
    std::vector<int> unrolled_hit_flag;
    std::tie(unrolled_pitches, unrolled_yaws, unrolled_pts, unrolled_hit_flag)
	= ray_dirn_server.fitDetailToPts(ray_origin, packet_pts);

    std::cout << "unrolled pitches: " << std::endl;
    dispVec(unrolled_pitches);
    std::cout << "unrolled yaws: " << std::endl;
    dispVec(unrolled_yaws);
    std::cout << "unrolled pts: " << std::endl;
    dispMat(unrolled_pts);
    std::cout << "unrolled hit flag: " << std::endl;
    dispVec(unrolled_hit_flag);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}