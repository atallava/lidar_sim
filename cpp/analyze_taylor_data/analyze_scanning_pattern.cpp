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
	{0.5564,1.4687,-0.9314},
	{1.0495,2.7701,-0.4867},
	{0.5659,1.4936,-0.8974},
	{1.169,3.0855,-0.4637},
	{0.5618,1.4829,-0.8432},
	{0.5965,1.5744,-0.8456},
	{1.1867,3.1322,-0.3125},
	{0.6007,1.5857,-0.8026},
	{1.1105,2.931,-0.2192},
	{0.6027,1.5907,-0.7573},
	{0.9216,2.4324,-0.1213},
	{0.6447,1.7016,-0.7601},
	{0.9032,2.384,-0.0592},
	{0.6772,1.7875,-0.7464},
	{0.9169,2.4201,0},
	{0.6519,1.7206,-0.6697},
	{1.0095,2.6644,0.0662},
	{0.6646,1.7541,-0.6338},
	{0.9846,2.5987,0.1296},
	{1.0694,2.8227,-0.9419},
	{1.0377,2.7389,0.2048},
	{0.6702,1.7691,-0.5425},
	{0.9969,2.6313,0.2625},
	{0.6635,1.7514,-0.4903},
	{0.9867,2.6044,0.3257},
	{0.7316,1.9309,-0.4892},
	{2.1738,5.7377,0.8623},
	{0.6979,1.8422,-0.4187},
	{1.8403,4.8574,0.8534},
	{0.7012,1.8508,-0.3729},
	{1.8989,5.012,1.0098},
	{0.554,1.4494,-0.9202},
	{1.0782,2.8205,-0.4961},
	{0.5746,1.5033,-0.9042},
	{1.1668,3.0525,-0.4593},
	{0.57,1.4911,-0.8488},
	{0.5807,1.5192,-0.8169},
	{1.2081,3.1603,-0.3156},
	{0.5971,1.5619,-0.7915},
	{1.1412,2.9855,-0.2235},
	{0.6139,1.606,-0.7655},
	{1.0864,2.8421,-0.1419},
	{0.6076,1.5894,-0.7107},
	{0.9053,2.3682,-0.0589},
	{0.6419,1.6793,-0.702},
	{0.9177,2.4006,0},
	{0.6556,1.7151,-0.6683},
	{1.0102,2.6427,0.0657},
	{0.6657,1.7415,-0.63},
	{0.9744,2.5491,0.1273},
	{1.058,2.7678,-0.9246},
	{1.0558,2.7619,0.2068},
	{1.0004,2.6171,0.2614},
	{0.6618,1.7314,-0.4852},
	{0.9774,2.5569,0.3201},
	{0.7338,1.9196,-0.4869},
	{2.217,5.7997,0.8726},
	{0.7034,1.8401,-0.4187},
	{1.8702,4.8925,0.8605},
	{0.7025,1.8377,-0.3707},
	{1.9131,5.0046,1.0094},
	{0.5552,1.4397,-0.9151},
	{0.5885,1.526,-0.9189},
	{1.1894,3.0841,-0.4646},
	{0.5719,1.4828,-0.845},
	{0.564,1.4624,-0.7873},
	{1.2662,3.283,-0.3283},
	{0.59,1.5298,-0.7761},
	{1.2025,3.118,-0.2337},
	{0.6252,1.6212,-0.7736},
	{1.1021,2.8575,-0.1428},
	{0.6056,1.5703,-0.703},
	{0.9123,2.3655,-0.0589},
	{0.6442,1.6704,-0.6991},
	{0.9269,2.4035,0},
	{0.6573,1.7044,-0.6649},
	{1.0116,2.6229,0.0653},
	{0.6668,1.7289,-0.6261},
	{0.982,2.5462,0.1273},
	{1.0647,2.7606,0.2069},
	{0.7271,1.8852,-0.5794},
	{1.001,2.5956,0.2595},
	{2.0379,5.2841,0.6623},
	{0.736,1.9084,-0.4846},
	{2.2356,5.7968,0.8732},
	{0.7096,1.8399,-0.4191},
	{1.867,4.8409,0.8524},
	{0.7093,1.8393,-0.3714},
	{1.9314,5.008,1.0113},
	{0.5626,1.4461,-0.9202},
	{0.5823,1.4966,-0.9023},
	{1.2108,3.1119,-0.4693},
	{0.5705,1.4663,-0.8366},
	{1.2864,3.3064,-0.4149},
	{0.5612,1.4424,-0.7774},
	{1.3235,3.4017,-0.3405},
	{0.5912,1.5196,-0.7718},
	{1.2385,3.1832,-0.2388},
	{0.6135,1.5767,-0.7533},
	{1.1736,3.0162,-0.1509},
	{0.6063,1.5582,-0.6984},
	{0.928,2.3851,-0.0594},
	{0.6749,1.7345,-0.7268},
	{0.9391,2.4137,0},
	{0.661,1.6989,-0.6635},
	{1.0143,2.6069,0.0649},
	{0.6699,1.7217,-0.6242},
	{0.9903,2.5452,0.1274},
	{1.0671,2.7425,0.2058},
	{0.7564,1.944,-0.5981},
	{1.0145,2.6074,0.261},
	{0.9057,2.3278,-0.6539},
	{2.0593,5.2928,0.6641},
	{0.7494,1.9261,-0.4897},
	{0.7157,1.8396,-0.4196},
	{1.877,4.8243,0.8505},
	{0.7205,1.8518,-0.3744},
	{1.962,5.0426,1.0194},
	{0.5645,1.4398,-0.9171},
	{0.7471,1.9053,-0.3362},
	{0.5811,1.4821,-0.8944},
     {1.2081,3.0811,-0.4651},
     {0.5776,1.4731,-0.8413},
     {1.2697,3.2383,-0.4068},
     {0.5709,1.4559,-0.7855},
     {1.3397,3.4168,-0.3424},
     {0.5899,1.5046,-0.765},
     {1.1398,2.9069,-0.2183},
     {0.6203,1.5819,-0.7565},
     {1.1428,2.9146,-0.146},
     {0.605,1.5429,-0.6922},
     {0.9591,2.446,-0.061},
     {0.7046,1.7969,-0.7537},
     {0.952,2.428,0},
     {0.6682,1.7042,-0.6663},
     {1.035,2.6396,0.0658},
     {0.6709,1.7111,-0.621},
     {1.0122,2.5816,0.1293},
     {0.7471,1.9054,-0.6386},
     {1.0466,2.6692,0.2005},
     {0.6835,1.7433,-0.5369},
     {1.0431,2.6604,0.2666},
     {0.909,2.3183,-0.6519},
     {1.108,2.8259,0.355},
     {0.6912,1.7629,-0.4487},
     {0.9919,2.5298,0.3819},
     {0.727,1.8541,-0.4233},
     {1.9264,4.9131,0.867},
     {0.7239,1.8463,-0.3736},
     {2.0311,5.1801,1.0483},
     {0.5663,1.4317,-0.9131},
     {0.606,1.5322,-0.9258},
     {1.2048,3.0462,-0.4604},
     {0.5806,1.4681,-0.8394},
     {1.2764,3.2271,-0.4058},
     {0.5817,1.4708,-0.7945},
     {1.3828,3.4961,-0.3508},
     {0.5931,1.4995,-0.7633},
     {1.2724,3.2171,-0.2419},
     {0.6545,1.6548,-0.7923},
     {1.1375,2.8759,-0.1442},
     {0.6041,1.5273,-0.6861},
     {0.9847,2.4896,-0.0622},
     {0.7106,1.7965,-0.7544},
     {0.9614,2.4308,0},
     {0.6705,1.6952,-0.6635},
     {1.0465,2.6458,0.0661},
     {0.6627,1.6756,-0.6089},
     {1.0919,2.7607,0.1384},
     {0.7528,1.9032,-0.6386},
     {1.0589,2.6772,0.2013},
     {1.084,2.7406,0.275},
     {0.9151,2.3137,-0.6514},
     {0.6872,1.7373,-0.4427},
     {0.9987,2.525,0.3816},
     {0.7404,1.8719,-0.4279},
     {1.9301,4.8798,0.8622},
     {0.7207,1.8222,-0.3692},
     {2.1,5.3093,1.0757},
     {0.5696,1.4285,-0.912},
     {0.7566,1.8973,-0.3356},
     {0.6122,1.5354,-0.9287},
     {1.2082,3.0299,-0.4584},
     {0.5854,1.4681,-0.8404},
     {1.2832,3.218,-0.4051},
     {0.5806,1.4559,-0.7873},
     {1.4346,3.5977,-0.3613},
     {0.5926,1.486,-0.7573},
     {1.291,3.2376,-0.2437},
     {0.6788,1.7022,-0.8159},
     {1.1499,2.8838,-0.1448},
     {0.607,1.5222,-0.6845},
     {1.105,2.771,-0.0693},
     {0.7142,1.7911,-0.7529},
     {1.0112,2.5358,0},
     {0.6787,1.7021,-0.6669},
     {1.0531,2.641,0.066},
     {0.6576,1.6491,-0.5999},
     {1.0981,2.7539,0.1383},
     {0.7086,1.777,-0.5969},
     {0.6865,1.7215,-0.5314},
     {1.0776,2.7024,0.2714},
     {0.6891,1.7282,-0.4408},
     {2.0526,5.1474,0.7788},
     {0.7434,1.8644,-0.4266},
     {1.9276,4.8341,0.855},
     {0.7265,1.822,-0.3696},
     {2.1293,5.34,1.0831},
     {0.5707,1.4188,-0.9069},
     {0.7777,1.9335,-0.3424},
     {0.6155,1.5303,-0.9268},
     {1.2187,3.03,-0.459},
     {0.5996,1.4909,-0.8544},
     {1.2942,3.2178,-0.4056},
     {0.5769,1.4343,-0.7765},
     {1.3494,3.3551,-0.3374},
     {0.5923,1.4725,-0.7513},
     {1.3259,3.2967,-0.2485},
     {0.6702,1.6663,-0.7996},
     {1.2174,3.0268,-0.1521},
     {0.6163,1.5324,-0.6899},
     {1.1594,2.8827,-0.0721},
     {0.7188,1.7872,-0.7522},
     {1.0351,2.5736,0},
     {0.6824,1.6966,-0.6656},
     {1.0617,2.6397,0.0661},
     {0.6618,1.6454,-0.5993},
     {1.1026,2.7414,0.1378},
     {0.6832,1.6987,-0.5713},
     {1.0951,2.7229,0.2052},
     {0.6937,1.7248,-0.5331},
     {1.0968,2.7269,0.2742},
     {1.1134,2.7682,0.3489},
     {0.6863,1.7062,-0.4358},
     {0.7504,1.8658,-0.4275},
     {1.9559,4.8631,0.8612},
     {0.7297,1.8143,-0.3685},
     {2.1826,5.4266,1.102},
     {0.5703,1.406,-0.8998},
     {0.8324,2.0519,-0.3638},
     {0.6154,1.5172,-0.9199},
     {1.2277,3.0264,-0.459},
     {0.608,1.4989,-0.8601},
     {1.3284,3.2747,-0.4133},
     {0.5778,1.4243,-0.772},
     {0.5919,1.459,-0.7453},
     {1.3252,3.2668,-0.2465},
     {0.6669,1.644,-0.7899},
     {1.1603,2.8603,-0.1439},
     {0.6216,1.5323,-0.6907},
     {1.174,2.8941,-0.0725},
     {0.7234,1.7833,-0.7515},
     {1.0435,2.5724,0},
     {0.6874,1.6945,-0.6656},
     {1.071,2.6403,0.0662},
     {0.6695,1.6504,-0.6018},
     {1.1085,2.7325,0.1375},
     {0.6832,1.6843,-0.5672},
     {1.1025,2.7177,0.2051},
     {0.693,1.7085,-0.5287},
     {1.1041,2.7218,0.274},
     {0.9491,2.3397,-0.661},
     {1.1208,2.763,0.3487},
     {0.6877,1.6952,-0.4335},
     {1.1182,2.7566,0.4181},
     {1.9696,4.8555,0.8609},
     {0.7373,1.8176,-0.3696},
     {2.1684,5.3454,1.0868},
     {0.5714,1.3981,-0.8957},
     {0.7892,1.931,-0.3427},
     {0.6036,1.4768,-0.8964},
     {1.2431,3.0415,-0.4618},
     {0.6039,1.4777,-0.8488},
     {0.5822,1.4244,-0.7729},
     {1.3862,3.3917,-0.3418},
     {0.5936,1.4525,-0.7427},
     {1.33,3.2541,-0.2458},
     {0.6608,1.6169,-0.7777},
     {0.6214,1.5204,-0.6861},
     {1.2186,2.9817,-0.0748},
     {0.7337,1.7952,-0.7573},
     {1.0464,2.5604,0},
     {0.6954,1.7014,-0.669},
     {1.0832,2.6504,0.0665},
     {0.6839,1.6732,-0.6108},
     {1.1005,2.6926,0.1357},
     {0.6884,1.6842,-0.5677},
     {1.1533,2.822,0.2132},
     {0.6939,1.6978,-0.5259},
     {1.0901,2.6673,0.2688},
     {0.926,2.2656,-0.6407},
     {2.19,5.3583,0.6769},
     {0.6906,1.6898,-0.4325},
     {2.1025,5.1443,0.781},
     {0.8437,2.0644,-0.474},
     {1.9674,4.8138,0.8544},
     {0.7428,1.8175,-0.3699},
     {2.172,5.3143,1.0817},
     {0.5821,1.4123,-0.9059},
     {0.746,1.81,-0.3216},
     {0.6007,1.4573,-0.8856},
     {1.246,3.0231,-0.4595},
     {0.6036,1.4645,-0.8422},
     {0.5877,1.426,-0.7747},
     {0.5938,1.4407,-0.7376},
     {1.3814,3.3516,-0.2535},
     {0.658,1.5963,-0.7687},
     {1.1648,2.826,-0.1425},
     {0.628,1.5237,-0.6884},
     {1.3037,3.1629,-0.0794},
     {0.7128,1.7293,-0.7304},
     {1.1188,2.7145,0},
     {0.7685,1.8644,-0.734},
     {1.1406,2.7673,0.0695},
     {0.6924,1.68,-0.614},
     {1.1123,2.6986,0.1361},
     {0.7326,1.7775,-0.5999},
     {1.1777,2.8573,0.2161},
     {0.704,1.7081,-0.5298},
     {1.0844,2.6309,0.2655},
     {0.9283,2.2521,-0.6377},
     {0.6941,1.6841,-0.4316},
     {2.0815,5.0502,0.7677},
     {0.8618,2.0908,-0.4807},
     {2.0072,4.8699,0.8654},
     {0.7587,1.8407,-0.3751},
     {2.2072,5.3551,1.0913},
     {0.5859,1.4089,-0.9049},
     {0.7457,1.7931,-0.3191},
     {0.6032,1.4506,-0.8827},
     {1.2631,3.0375,-0.4623},
     {0.6069,1.4593,-0.8404},
     {1.8153,4.3654,-0.5529},
     {0.6272,1.5083,-0.8205},
     {1.5117,3.6351,-0.3673},
     {0.6011,1.4455,-0.741},
     {1.3904,3.3436,-0.2532},
     {0.6595,1.5858,-0.7647},
     {1.1752,2.8261,-0.1427},
     {0.6363,1.5302,-0.6922},
     {1.3098,3.1496,-0.0792},
     {0.6567,1.5791,-0.6678},
     {0.7859,1.8898,-0.7449},
     {1.1316,2.7213,0.0684},
     {0.6999,1.683,-0.6159},
     {1.1023,2.6508,0.1339},
     {0.7477,1.7981,-0.6077},
     {1.1552,2.778,0.2104},
     {0.7116,1.7112,-0.5314},
     {1.0697,2.5723,0.2599},
     {2.2226,5.3448,0.6769},
     {0.7024,1.6891,-0.4335},
     {2.0548,4.9412,0.7521},
     {1.0501,2.5253,-0.5813},
     {2.0415,4.9092,0.8735},
     {0.7833,1.8837,-0.3844},
     {2.3032,5.5386,1.1302}};
    
    RayDirnServer ray_dirn_server;

    std::vector<double> unrolled_yaws;
    std::vector<double> unrolled_pitches;
    std::vector<std::vector<double> > unrolled_pts;
    std::vector<int> unrolled_hit_flag;
    std::tie(unrolled_pitches, unrolled_yaws, unrolled_pts, unrolled_hit_flag)
	= ray_dirn_server.fitDetailToPts(ray_origin, packet_pts);
    std::vector<std::vector<double> > unrolled_ray_dirns = calcRayDirnsFromSph(unrolled_yaws, unrolled_pitches);
    

    std::cout << "unrolled pitches: " << std::endl;
    dispVec(unrolled_pitches);
    std::cout << "unrolled yaws: " << std::endl;
    dispVec(unrolled_yaws);
    std::cout << "unrolled pts: " << std::endl;
    dispMat(unrolled_pts);
    std::cout << "unrolled hit flag: " << std::endl;
    dispVec(unrolled_hit_flag);
    std::cout << "unrolled ray dirns: " << std::endl;
    dispMat(unrolled_ray_dirns);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_time << "s." << std::endl;
	
    return(1);
}
