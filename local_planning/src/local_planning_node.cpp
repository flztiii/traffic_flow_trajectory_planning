#include "local_planning_node.hpp"

int main(int argv, char** argc) {
    ros::init(argv, argc, "local_planning_node");
    std::shared_ptr<LocalPlanningNode> local_planning_node_ptr = std::make_shared<LocalPlanningNode>();
    std::cout << "input anything to start" << std::endl;
    getchar();
    // std::string root_path = ros::package::getPath("local_planning") + "/record/";
    // if (std::filesystem::exists(std::filesystem::path(root_path))) {
    //     std::filesystem::remove_all(std::filesystem::path(root_path));
    // }
    // std::filesystem::create_directory(std::filesystem::path(root_path));
    // int test_times = 100;
    // int success_times = 0;
    // for (int test_count = 0; test_count < test_times; test_count++) {
    //     ErrorType test_result = local_planning_node_ptr->startPlanningOnce(test_count);
    //     std::string file_path = root_path + "state_record_" + std::to_string(test_count) + ".csv";
    //     std::ofstream data_file(file_path, std::ios::out|std::ios::app);
    //     if (test_result == kSuccess) {
    //         success_times++;
    //         data_file << "1";
    //     } else {
    //         data_file << "0";
    //     }
    //     data_file.close();
    // }
    // std::cout << "total test times: " << test_times << ", success times: " << success_times << std::endl;
    ErrorType test_result = local_planning_node_ptr->startGymPlanningOnce();
    if (test_result == kSuccess) {
        std::cout << "test success" << std::endl;
    } else {
        std::cout << "test failed" << std::endl;
    }
    return 0;
}