#include <memory>
#include <algorithm>
#include<vector>
#include<string>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class Discharge : public plansys2::ActionExecutorClient
{
public:
  Discharge()
  : plansys2::ActionExecutorClient("discharge", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  { std :: vector <std :: string > arguments = get_arguments () ;
    if (progress_ < 1.0) {
      progress_ += 0.2;
      send_feedback(progress_, "Agent "+arguments [1]+ " is unloading "+ 
            arguments [2]+ " on vehicle "+arguments [4]+ " at "+
            arguments [0] + " using place "+arguments [3] + " at loc "+ arguments [0]);
    } else {
      finish(true, 1.0, "Agent "+arguments [1]+ " unloaded "+
                arguments [2]+ " on vehicle "+arguments [4]+ " at "+
                arguments [0] + " using place "+arguments [3]);

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "\r\e[K" << std::flush;
    std::cout << "Agent "+arguments [1]+ " is unloading "+arguments
            [2]+ " on vehicle "+arguments [4]+ " at "+arguments [0] + " using place "+
            arguments [3] +" . . . [ " << std :: min( 100.0 ,
            progress_ * 100.0) << "% ] " <<
            std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Discharge>();

  node->set_parameter(rclcpp::Parameter("action_name", "DISCHARGE"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
