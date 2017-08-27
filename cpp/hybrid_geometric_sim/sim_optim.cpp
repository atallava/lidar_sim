#include <ostream>
#include <ctime>

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
    // set up optim assistant
    optim_assistant.m_verbose = 1;
    // section id
    optim_assistant.m_section_id_for_model = 3;
    // which non ground blocks to build
    optim_assistant.m_non_ground_block_ids = std::vector<int> {16,17,18,19};
    // which ground blocks to use
    optim_assistant.m_ground_block_ids = std::vector<int> {2};
    // path of section to use
    optim_assistant.m_section_id_for_sim = 3;
    // section packet ids
    optim_assistant.m_section_packet_start = 1;
    optim_assistant.m_section_packet_end = 10;
    optim_assistant.m_section_packet_skip = 5;
    optim_assistant.init();

    // set up nlopt
    // x = {n_cluster_per_pt, max_maha_dist_for_hit}
    nlopt::opt opt(nlopt::LN_COBYLA, 2); // algo and dimensionality
    std::vector<double> lb(2), ub(2);
    lb[0] = 0.001; lb[1] = 0.1;
    opt.set_lower_bounds(lb);
    ub[0] = 0.05; ub[2] = 6;

    opt.set_min_objective(myfunc, NULL); 

    int max_eval = 1;
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
