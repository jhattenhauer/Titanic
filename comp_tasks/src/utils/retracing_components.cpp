#include "utils/retracing_components.hpp"

using namespace std::chrono_literals;

namespace comp_tasks
{
  Retrace::Retrace(const rclcpp::NodeOptions & options)
  : Task(options, "Retrace")
  {
    
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Retrace::on_configure(const rclcpp_lifecycle::State &)
  {
    return result;
  }

  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params)
  {
    return;
  } 

  void Retrace::setState(std::string str_state)
  {
    return;
  }


  void Retrace::executeRecoveryBehaviour() //Overwritten recovery behaviour function
  {
    return;
  }

  void Retrace::taskLogic(const int& standin)
  {
    return;
  }

  void checkIfFinished()
  {
    return;
  }

}
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(comp_tasks::Retrace)
