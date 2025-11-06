/*
 需求：将ROS2中常用的速度指令（geometry_msgs::msg::Twist），
 转换成四组机器人的速度指令（unitree_api::msg::Request）。

 流程：
 1.包含头文件；
 2.初始化ROS2客户端；
 3.自定义节点类；
  3-1.创建四组机器人速度指令发布对象；
  3-2.创建ROS2速度指令订阅对象；
  3-3.在订阅对象的回调函数中将Twist转换成Request并发布。
 4.调用spin函数，并传入节点对象指针；
 5.资源释放。
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"

using namespace std::placeholders;

// 3.自定义节点类；
class Twist2Request: public rclcpp::Node{
public:
  Twist2Request():Node("twist2request_node"){
    RCLCPP_INFO(this->get_logger(),"convert geometry_msgs::msg::Twist to unitree_api::msg::Request");
    // 3-1.创建四组机器人速度指令发布对象；
    req_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request",10);
    // 3-2.创建ROS2速度指令订阅对象；
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
                  ("cmd_vel",10,std::bind(&Twist2Request::twist_to_request,this,_1));
    // 3-3.在订阅对象的回调函数中将Twist转换成Request并发布。
  }
private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  void twist_to_request(const geometry_msgs::msg::Twist::SharedPtr twist){
      unitree_api::msg::Request request;
      auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
      // 转换（只需要x、y的线速度和z的角速度）
      double x = twist->linear.x;
      double y = twist->linear.y;
      double th = twist->angular.z;
      if (x != 0 || y != 0 || th != 0)
      {
          api_id = ROBOT_SPORT_API_ID_MOVE;

          // 设置线速度与角速度
          nlohmann::json js;
          js["x"] = x;
          js["y"] = y;
          js["z"] = th;
          request.parameter = js.dump();
          RCLCPP_INFO(this->get_logger(),"current speed: %s",request.parameter.c_str());
      }

      request.header.identity.api_id = api_id;
      req_pub_->publish(request);
  }
};


int main(int argc, char const *argv[])
{
  // 2.初始化ROS2客户端；
  rclcpp::init(argc,argv);
  // 4.调用spin函数，并传入节点对象指针；
  rclcpp::spin(std::make_shared<Twist2Request>());
  // 5.资源释放。
  rclcpp::shutdown();
  return 0;
}