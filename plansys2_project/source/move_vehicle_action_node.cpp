#include <memory>
#include <algorithm>
#include<vector>
#include<string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveVehicle : public plansys2::ActionExecutorClient
{
public:
  MoveVehicle()
  : plansys2::ActionExecutorClient("move_vehicle", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  { std :: vector <std :: string > arguments = get_arguments () ;
    if (progress_ < 1.0) {
      progress_ += 0.2;
      send_feedback(progress_, "Agent "+arguments [0]+ " moving on "+ 
            arguments [2]+ " from "+ arguments [1] + " to "+
            arguments [1]);
    } else {
      finish(true, 1.0, "Agent "+arguments [0]+ " moved "+ 
            arguments [2]+ " from "+ arguments [1] + " to "+
            arguments [1]);

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Agent "+arguments [0]+ " moving on "+ 
            arguments [2]+ " from "+ arguments [1] + " to "+
            arguments [1] +" . . . [ " << std::min(100.0, progress_ * 100.0) << "% ]  " <<
            std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveVehicle>();

  node->set_parameter(rclcpp::Parameter("action_name", "MOVE_VEHICLE"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
