#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <fstream>

#include <plansys2_msgs/srv/add_problem.hpp>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TransitController : public rclcpp::Node
{
    public:
        TransitController() : rclcpp::Node("transit_controller"), state_(STARTING)
        {
            this->declare_parameter("goal", "(and (piece_at workpiece1 station3) (piece_at workpiece2 station3) (piece_at workpiece3 station3) (robot_available robot1) (robot_available robot2))");
            this->declare_parameter("problem_file_path", "/home/mdh-es/multirobot_ws/src/jigless-planner/pddl/transit_problem.pddl");
            add_problem_client_ = this->create_client<plansys2_msgs::srv::AddProblem>("/problem_expert/add_problem");
        };

        void init()
        {
            domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
            planner_client_ = std::make_shared<plansys2::PlannerClient>();
            problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
            executor_client_ = std::make_shared<plansys2::ExecutorClient>();
            init_knowledge();
        };

        void init_knowledge()
        {
            std::string problem_file_path = this->get_parameter("problem_file_path").as_string();
            std::ifstream problem_file(problem_file_path);
            if (!problem_file.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open problem file: %s", problem_file_path.c_str());
                return;
            }

            std::stringstream buffer;
            buffer << problem_file.rdbuf();
            auto request = std::make_shared<plansys2_msgs::srv::AddProblem::Request>();
            request->problem = buffer.str();
            problem_file.close();
            while(!add_problem_client_->wait_for_service(std::chrono::seconds(1))){
                if (!rclcpp::ok()){
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto result = add_problem_client_->async_send_request(request, std::bind(&TransitController::response_callback, this, std::placeholders::_1));
        };

        bool response_callback(rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedFuture future)
        {
            auto status = future.wait_for(std::chrono::seconds(1));
            if (status == std::future_status::ready){
                auto result = future.get();
                if (result->success){
                    RCLCPP_INFO(this->get_logger(), "Problem added");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Error adding problem");
                }
                return result->success;
            }
            return false;
        };

        void step() {
            problem_expert_->setGoal(plansys2::Goal(this->get_parameter("goal").as_string()));

            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl);
                return;
            }

            if (executor_client_->start_plan_execution(plan.value())) {
                RCLCPP_INFO(this->get_logger(), "Plan started");
            }
        };

        private:
            typedef enum
            {
                STARTING
            } StateType;
            StateType state_;
            std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
            std::shared_ptr<plansys2::PlannerClient> planner_client_;
            std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
            std::shared_ptr<plansys2::ExecutorClient> executor_client_;
            rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr add_problem_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransitController>();
    node->init();
    node->step();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}