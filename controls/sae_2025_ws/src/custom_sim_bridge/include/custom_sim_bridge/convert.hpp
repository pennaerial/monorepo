#include "sim_interfaces/srv/attach_detach.hpp"
#include "gz/custom_msgs/dynamic_detachable_joint.pb.h"


gz::custom_msgs::AttachDetachRequest convert_to_gz(sim_interfaces::srv::AttachDetach::Request::SharedPtr ros_req) {
    gz::custom_msgs::AttachDetachRequest gz_req;
    gz_req.set_child_model_name(ros_req->child_model_name);
    gz_req.set_child_link_name(ros_req->child_link_name);
    gz_req.set_command(ros_req->command);
    return gz_req;
}


sim_interfaces::srv::AttachDetach::Response::SharedPtr convert_to_ros(gz::custom_msgs::AttachDetachResponse gz_resp) {
    auto ros_resp = std::make_shared<sim_interfaces::srv::AttachDetach::Response>();
    ros_resp->message = gz_resp.message();
    ros_resp->success = gz_resp.success();
    return ros_resp;
}
