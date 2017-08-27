#include <ostream>
#include <ctime>

#include <nlopt.hpp>

#include <lidar_sim/DataProcessingUtils.h>
#include <lidar_sim/OptimProgress.h>

using namespace lidar_sim;

clock_t start_time = clock();
int fn_eval_count = 0;
OptimProgress optim_progress;

void dispProgressMsg(const std::vector<double>& x, const double J, const double elapsed_time = -1)
{
    std::cout << "fn eval count: " << fn_eval_count 
	      << ", x: " << getStrFromVec(x) << ", J: " << J;
    if (elapsed_time == -1)
	std::cout << std::endl;
    else
	std::cout << ", elapsed time: " << elapsed_time << std::endl;
}

double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    fn_eval_count++;
    double J = x[0];
    double elapsed_time = (clock()-start_time)/CLOCKS_PER_SEC;

    // log progress
    dispProgressMsg(x, J, elapsed_time);
    optim_progress.log(x, J, elapsed_time);

    return J;
}

int main(int argc, char **argv)
{
    // x = {n_cluster_per_pt, max_maha_dist_for_hit}
    nlopt::opt opt(nlopt::LN_COBYLA, 2); // algo and dimensionality
    std::vector<double> lb(2), ub(2);
    lb[0] = 0.001; lb[1] = 0.1;
    opt.set_lower_bounds(lb);
    ub[0] = 0.05; ub[2] = 6;

    opt.set_min_objective(myfunc, NULL); 

    int max_eval = 100;
    opt.set_maxeval(max_eval);

    opt.set_xtol_rel(1e-4); // relative tolerance on x

    std::vector<double> x(2);
    x[0] = 0.0146; x[1] = 3.5;
    double minf;
    nlopt::result result = opt.optimize(x, minf);

    std::cout << "result: " << std::endl;
    dispProgressMsg(x, minf);

    std::string rel_path_optim_progress = "data/hg_optim/optim_progress.txt";
    optim_progress.save(rel_path_optim_progress);
}
