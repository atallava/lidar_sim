#include <ostream>
#include <ctime>
#include <iomanip>

#include <nlopt.hpp>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/OptimProgress.h>
#include <lidar_sim/OptimAssistant.h>

using namespace lidar_sim;

// todo: is there a cleaner way than making these global?
clock_t start_time = clock();
int fn_eval_count = 0;
OptimProgress optim_progress;
OptimAssistant optim_assistant;

void dispProgressMsg(const std::vector<double>& x, const double obj, const double elapsed_time = -1)
{
    std::cout << "fn eval count: " << fn_eval_count 
	      << ", x: " << getStrFromVec(x) << ", obj: " << obj;
    if (elapsed_time == -1)
	std::cout << std::endl;
    else
	std::cout << ", elapsed time: " << elapsed_time << std::endl;
}

double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    fn_eval_count++;
    double obj = optim_assistant.calcObj(x);
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;

    // log progress
    dispProgressMsg(x, obj, elapsed_time);
    optim_progress.log(x, obj, elapsed_time);

    return obj;
}

int main(int argc, char **argv)
{
    // todo: delete
    std::cout << getDateString("%M%d%m%y") << std::endl;;
    std::exit(0);

    // set up optim assistant
    optim_assistant.m_verbose = 1;
    // section idx for modeling
    optim_assistant.m_section_id_for_model = 3;

    // non ground block idx
    optim_assistant.m_non_ground_block_ids = std::vector<int> {16,17,18,19};
    // ground block idx. currently not remodeled
    optim_assistant.m_ground_block_ids = std::vector<int> {2};

    // non ground block idx
    // section idx for sim
    optim_assistant.m_section_id_for_sim = 3;
    // section packet ids
    optim_assistant.m_section_packet_start = 0;
    // todo: get automatically
    optim_assistant.m_section_packet_end = 3975; 
    optim_assistant.m_section_packet_step = 10;

    // sim type
    optim_assistant.m_sim_type = 1; // slice sim
    optim_assistant.init();

    // set up nlopt
    // x = {n_cluster_per_pt, max_maha_dist_for_hit}
    nlopt::opt opt(nlopt::LN_COBYLA, 2); // algo and dimensionality
    std::vector<double> lb(2), ub(2);
    lb[0] = 0.001; lb[1] = 0.01;
    opt.set_lower_bounds(lb);
    ub[0] = 0.05; ub[2] = 5;

    opt.set_min_objective(myfunc, NULL); 

    // todo: increase max eval
    int max_eval = 180;
    opt.set_maxeval(max_eval);

    opt.set_xtol_rel(1e-4); // relative tolerance on x

    std::vector<double> x(2);
    x[0] = 0.0146; x[1] = 3.5;
    double minf;

    // fire away
    nlopt::result result = opt.optimize(x, minf);

    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;
    std::cout << "result: " << std::endl;
    dispProgressMsg(x, minf, elapsed_time);

    std::string rel_path_optim_progress = "data/sim_optim/optim_progress.txt";
    optim_progress.save(rel_path_optim_progress);
}
