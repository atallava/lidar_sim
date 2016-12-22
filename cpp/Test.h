#pragma once
#include <string>

namespace ol{
    class Test{
    public:
        bool testVizPCD(std::string file_name);
        bool testVizPoints();
        bool testDataset(std::string file_name);
        bool validatePredictor(std::string file_name, std::string predictor_type,
			       double predictor_param,
			       bool adjust_for_under_represented_classes,
			       int num_training_passes);
        bool validatePredictor(std::string train_file_name, std::string test_file_name,
			       std::string predictor_type, double predictor_param,
			       bool adjust_for_under_represented_classes,
			       int num_training_passes);
    };
}
