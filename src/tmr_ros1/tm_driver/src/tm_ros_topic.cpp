#include "tm_driver/tm_ros_node.h"

////////////////////////////////
// Topic
////////////////////////////////

void TmRosNode::publish_fbs()
{
    PubMsg &pm = pm_;
    TmRobotState &state = iface_.state;

    // Publish feedback state
    pm.fbs_msg.header.stamp = ros::Time::now();

    pm.fbs_msg.is_svr_connected = iface_.svr.is_connected();
    pm.fbs_msg.is_sct_connected = iface_.sct.is_connected();

    pm.fbs_msg.joint_pos = state.joint_angle();
    pm.fbs_msg.joint_vel = state.joint_speed();
    pm.fbs_msg.joint_tor = state.joint_torque();
    pm.fbs_msg.tool_pose = state.tool_pose();
    pm.fbs_msg.tcp_speed = state.tcp_speed_vec();
    pm.fbs_msg.tcp_force = state.tcp_force_vec();
    pm.fbs_msg.robot_link = state.is_linked();
    pm.fbs_msg.is_data_table_correct = state.is_data_table_correct();
    pm.fbs_msg.robot_error = state.has_error();
    pm.fbs_msg.project_run = state.is_project_running();
    pm.fbs_msg.project_pause = state.is_project_paused();
    pm.fbs_msg.safetyguard_a = state.is_safeguard_A();
    pm.fbs_msg.e_stop = state.is_EStop();
    pm.fbs_msg.camera_light = state.camera_light();
    pm.fbs_msg.error_code = state.error_code();
    pm.fbs_msg.project_speed = state.project_speed();
    pm.fbs_msg.ma_mode = state.ma_mode();
    pm.fbs_msg.robot_light = state.robot_light();
    pm.fbs_msg.cb_digital_output = state.ctrller_DO();
    pm.fbs_msg.cb_digital_input = state.ctrller_DI();
    pm.fbs_msg.cb_analog_output = state.ctrller_AO();
    pm.fbs_msg.cb_analog_input = state.ctrller_AI();
    pm.fbs_msg.ee_digital_output = state.ee_DO();
    pm.fbs_msg.ee_digital_input = state.ee_DI();
    //pm.fbs_msg.ee_analog_output = state.ee_AO();
    pm.fbs_msg.ee_analog_input = state.ee_AI();
    pm.fbs_msg.error_content = state.error_content();
    pm.fbs_pub.publish(pm.fbs_msg);

    // Publish joint state
    pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
    pm.joint_msg.position = pm.fbs_msg.joint_pos;
    pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
    pm.joint_msg.effort = pm.fbs_msg.joint_tor;
    pm.joint_pub.publish(pm.joint_msg);

    // Publish tool pose
    //TmPoseConversion::msg_from_vec(pm.tool_pose_msg.pose, pm.fbs_msg.tool_pose);
    auto &pose = pm.fbs_msg.tool_pose;
    tf::Quaternion quat;
    quat.setRPY(pose[3], pose[4], pose[5]);
    tf::Transform Tbt(quat, tf::Vector3(pose[0], pose[1], pose[2]));
    tf::poseTFToMsg(Tbt, pm.tool_pose_msg.pose);
    pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
    pm.tool_pose_msg.header.frame_id = base_frame_name_;
    /*pm.tool_pose_msg.pose.position.x = pose[0];
    pm.tool_pose_msg.pose.position.y = pose[1];
    pm.tool_pose_msg.pose.position.z = pose[2];
    pm.tool_pose_msg.pose.orientation.x = quat.x();
    pm.tool_pose_msg.pose.orientation.y = quat.y();
    pm.tool_pose_msg.pose.orientation.z = quat.z();
    pm.tool_pose_msg.pose.orientation.w = quat.w();*/
    pm.tool_pose_pub.publish(pm.tool_pose_msg);

    // Boardcast transform (tool pose)
    //TmPoseConversion::tf_from_vec(pm.transform, pm.fbs_msg.tool_pose);
    //pm.transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
    //pm.transform.setRotation(quat);
    pm.tfbc.sendTransform(tf::StampedTransform(
        Tbt, pm.joint_msg.header.stamp, base_frame_name_, tool_frame_name_));

}
void TmRosNode::publish_svr()
{
    PubMsg &pm = pm_;
    TmSvrData &data = iface_.svr.data;
    {
        boost::unique_lock<boost::mutex> lck(svr_mtx_);
        pm.svr_msg.id = data.transaction_id();
        pm.svr_msg.mode = (int)(data.mode());
        pm.svr_msg.content = std::string{ data.content(), data.content_len() };
        pm.svr_msg.error_code = (int)(data.error_code());
        svr_updated_ = true;
    }
    svr_cond_.notify_all();

    print_info("TM_ROS: (TM_SVR): (%s) (%d) %s",
        pm.svr_msg.id.c_str(), pm.svr_msg.mode, pm.svr_msg.content.c_str());

    pm.svr_msg.header.stamp = ros::Time::now();
    pm.svr_pub.publish(pm.svr_msg);
}
bool TmRosNode::publish_func()
{
    //PubMsg &pm = pm_;
    TmSvrCommunication &svr = iface_.svr;
    int n;
    auto rc = svr.recv_spin_once(1000, &n);
    if (rc == TmCommRC::ERR ||
        rc == TmCommRC::NOTREADY ||
        rc == TmCommRC::NOTCONNECT) {
        return false;
    }
    else if (rc != TmCommRC::OK) {
        return true;
    }
    bool fbs = false;
    std::vector<TmPacket> &pack_vec = svr.packet_list();

    for (auto &pack : pack_vec) {
        if (pack.type == TmPacket::Header::CPERR) {
            print_info("TM_ROS: (TM_SVR): CPERR");
            svr.err_data.set_CPError(pack.data.data(), pack.data.size());
            print_error(svr.err_data.error_code_str().c_str());

            // cpe response

        }
        else if (pack.type == TmPacket::Header::TMSVR) {

            svr.err_data.error_code(TmCPError::Code::Ok);

            //TODO ? lock and copy for service response
            TmSvrData::build_TmSvrData(svr.data, pack.data.data(), pack.data.size(), TmSvrData::SrcType::Shallow);

            if (svr.data.is_valid()) {
                switch (svr.data.mode()) {
                case TmSvrData::Mode::RESPONSE:
                    //print_info("TM_ROS: (TM_SVR): (%s) RESPONSE [%d]",
                    //    svr.data.transaction_id().c_str(), (int)(svr.data.error_code()));
                    publish_svr();
                    break;
                case TmSvrData::Mode::BINARY:
                    svr.state.mtx_deserialize(svr.data.content(), svr.data.content_len());
                    fbs = true;
                    break;
                case TmSvrData::Mode::READ_STRING:
                case TmSvrData::Mode::READ_JSON:
                    publish_svr();
                    break;
                default:
                    print_info("TM_ROS: (TM_SVR): (%s): invalid mode (%d)",
                        svr.data.transaction_id().c_str(), (int)(svr.data.mode()));
                    break;
                }
            }
            else {
                print_info("TM_ROS: (TM_SVR): invalid data");
            }
        }
        else {
            print_info("TM_ROS: (TM_SVR): invalid header");
        }
    }
    if (fbs) {
        publish_fbs();
    }
    return true;
}
void TmRosNode::publisher()
{
    PubMsg &pm = pm_;
    TmSvrCommunication &svr = iface_.svr;

    print_info("TM_ROS: publisher thread begin");

    //PubMsg pm;
    //pm.fbs_pub = nh_.advertise<tm_msgs::FeedbackState>("feedback_states", 1);
    //pm.joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    //pm.tool_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("tool_pose", 1);

    //pm.svr_pub = nh_.advertise<tm_msgs::SvrResponse>("tm_driver/svr_response", 1);

    pm.joint_msg.name = joint_names_;
    pm.joint_msg.position.assign(joint_names_.size(), 0.0);
    pm.joint_msg.velocity.assign(joint_names_.size(), 0.0);
    pm.joint_msg.effort.assign(joint_names_.size(), 0.0);

    while (ros::ok()) {
        //bool reconnect = false;
        if (!svr.recv_init()) {
            print_info("TM_ROS: (TM_SVR): is not connected");
        }
        while (ros::ok() && svr.is_connected()) {
            if (!publish_func()) break;
        }
        svr.Close();

        // reconnect == true
        if (!ros::ok()) break;
        if (pub_reconnect_timeval_ms_ <= 0) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        }
        print_info("TM_ROS: (TM_SVR): reconnect in ");
        int cnt = 0;
        while (ros::ok() && cnt < pub_reconnect_timeval_ms_) {
            if (cnt % 500 == 0) {
                print_info("%.1f sec...", 0.001 * (pub_reconnect_timeval_ms_ - cnt));
            }
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
            ++cnt;
        }
        if (ros::ok() && pub_reconnect_timeval_ms_ >= 0) {
            print_info("0 sec\nTM_ROS: (TM_SVR): connect(%d)...", pub_reconnect_timeout_ms_);
            svr.Connect(pub_reconnect_timeout_ms_);
        }
    }
    svr.Close();
    printf("TM_ROS: publisher thread end\n");
}

void TmRosNode::sct_msg()
{
    SctMsg &sm = sm_;
    TmSctData &data = iface_.sct.sct_data;

    sm.sct_msg.id = data.script_id();
    sm.sct_msg.script = std::string{ data.script(), data.script_len() };

    if (data.has_error()) {
        print_info("TM_ROS: (TM_SCT): err: (%s): %s", sm.sct_msg.id.c_str(), sm.sct_msg.script.c_str());
    }
    else {
        print_info("TM_ROS: (TM_SCT): res: (%s): %s", sm.sct_msg.id.c_str(), sm.sct_msg.script.c_str());
    }

    sm.sct_msg.header.stamp = ros::Time::now();
    sm.sct_pub.publish(sm.sct_msg);
}
void TmRosNode::sta_msg()
{
    SctMsg &sm = sm_;
    TmStaData &data = iface_.sct.sta_data;
    {
        boost::lock_guard<boost::mutex> lck(sta_mtx_);
        sm.sta_msg.subcmd = data.subcmd_str();
        sm.sta_msg.subdata = std::string{ data.subdata(), data.subdata_len() };
        sta_updated_ = true;
    }
    sta_cond_.notify_all();

    print_info("TM_ROS: (TM_STA): res: (%s): %s", sm.sta_msg.subcmd.c_str(), sm.sta_msg.subdata.c_str());

    sm.sta_msg.header.stamp = ros::Time::now();
    sm.sta_pub.publish(sm.sta_msg);
}
bool TmRosNode::sct_func()
{
    TmSctCommunication &sct = iface_.sct;
    int n;
    auto rc = sct.recv_spin_once(1000, &n);
    if (rc == TmCommRC::ERR ||
        rc == TmCommRC::NOTREADY ||
        rc == TmCommRC::NOTCONNECT) {
        return false;
    }
    else if (rc != TmCommRC::OK) {
        return true;
    }
    std::vector<TmPacket> &pack_vec = sct.packet_list();

    for (auto &pack : pack_vec) {
        switch (pack.type) {
        case TmPacket::Header::CPERR:
            print_info("TM_ROS: (TM_SCT): CPERR");
            sct.err_data.set_CPError(pack.data.data(), pack.data.size());
            print_error(sct.err_data.error_code_str().c_str());

            // cpe response

            break;

        case TmPacket::Header::TMSCT:

            sct.err_data.error_code(TmCPError::Code::Ok);

            //TODO ? lock and copy for service response
            TmSctData::build_TmSctData(sct.sct_data, pack.data.data(), pack.data.size(), TmSctData::SrcType::Shallow);

            sct_msg();
            break;

        case TmPacket::Header::TMSTA:

            sct.err_data.error_code(TmCPError::Code::Ok);

            TmStaData::build_TmStaData(sct.sta_data, pack.data.data(), pack.data.size(), TmStaData::SrcType::Shallow);

            sta_msg();
            break;

        default:
            print_info("TM_ROS: (TM_SCT): invalid header");
            break;
        }
    }
    return true;
}
void TmRosNode::sct_responsor()
{
    SctMsg &sm = sm_;
    TmSctCommunication &sct = iface_.sct;

    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

    print_info("TM_ROS: sct_response thread begin");

    //SctMsg sm;
    //sm.sct_pub = nh_.advertise<tm_msgs::SctResponse>("tm_driver/sct_response", 1);
    //sm.sta_pub = nh_.advertise<tm_msgs::StaResponse>("tm_driver/sta_response", 1);

    while (ros::ok()) {
        //bool reconnect = false;
        if (!sct.recv_init()) {
            print_info("TM_ROS: (TM_SCT): is not connected");
        }
        while (ros::ok() && sct.is_connected()) {
            if (!sct_func()) break;
        }
        sct.Close();

        // reconnect == true
        if (!ros::ok()) break;
        if (sct_reconnect_timeval_ms_ <= 0) {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        }
        print_info("TM_ROS: (TM_SCT) reconnect in ");
        int cnt = 0;
        while (ros::ok() && cnt < sct_reconnect_timeval_ms_) {
            if (cnt % 1000 == 0) {
                print_info("%.1f sec...", 0.001 * (sct_reconnect_timeval_ms_ - cnt));
            }
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
            ++cnt;
        }
        if (ros::ok() && sct_reconnect_timeval_ms_ >= 0) {
            print_info("0 sec\nTM_ROS: (TM_SCT) connect(%d)...", sct_reconnect_timeout_ms_);
            sct.Connect(sct_reconnect_timeout_ms_);
        }
    }
    sct.Close();
    printf("TM_ROS: sct_response thread end\n");
}
