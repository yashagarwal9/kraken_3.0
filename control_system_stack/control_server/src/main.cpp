#include<ros/ros.h>
#include <control_server/ControlServer.h>
#include "resources/topicHeader.h"
#include "control_server/loadParam.h"
#include <dynamic_reconfigure/server.h>
#include <control_server/paramsConfig.h>
int main(int argc,char** argv)
{
    ros::init(argc,argv,"Control_Server");
    ros::NodeHandle n;
    ros::NodeHandle n_a("~/left_surge");
    ros::NodeHandle n_b("~/right_surge");
    ros::NodeHandle n_c("~/front_sway");
    ros::NodeHandle n_d("~/back_sway");
    ros::NodeHandle n_e("~/front_heave");
    ros::NodeHandle n_f("~/back_heave");

    if(argc>2)
    {
        std::cerr<<atoi(argv[1])<<std::endl;
        float freq = atof(argv[1]) ;
        kraken_controller::ControlServer _server(freq);
        std::vector<std::string> _files;

        for(int i=2; i<argc; i++)
        {
            _files.push_back(argv[i]);
        }

        _server.loadParams(_files);
        actionlib::SimpleActionServer<kraken_msgs::advancedControllerAction> _ser1(n,topics::CONTROL_ADVANCEDCONTROLLER_ACTION/*ros::this_node::getName()*/,boost::bind(&kraken_controller::ControlServer::executePoseChange, &_server, _1),false);
        actionlib::SimpleActionServer<kraken_msgs::controllerAction> _ser2(n,topics::CONTROL_SETPOINT_ACTION/*ros::this_node::getName()*/,boost::bind(&kraken_controller::ControlServer::executeOrientationChange, &_server, _1),false);
        ros::ServiceServer service1 = n.advertiseService(topics::CONTROL_MOVEALONG_SERV,&kraken_controller::ControlServer::moveAlongLine,&_server);
        ros::ServiceServer service2 = n.advertiseService(topics::CONTROL_SWITCH_CONTROL,&kraken_controller::ControlServer::changeController,&_server);
        ros::ServiceServer service3 = n.advertiseService(topics::CONTROL_LOADPARAM,&kraken_controller::ControlServer::loadParamsCB,&_server);
        dynamic_reconfigure::Server<control_server::paramsConfig> thruster0(n_a);
        dynamic_reconfigure::Server<control_server::paramsConfig> thruster1(n_b);
        dynamic_reconfigure::Server<control_server::paramsConfig> thruster2(n_c);
        dynamic_reconfigure::Server<control_server::paramsConfig> thruster3(n_d);
        dynamic_reconfigure::Server<control_server::paramsConfig> thruster4(n_e);
        dynamic_reconfigure::Server<control_server::paramsConfig> thruster5(n_f);
        thruster0.setCallback(boost::bind(&kraken_controller::ControlServer::callback0, &_server, _1, _2));
        thruster1.setCallback(boost::bind(&kraken_controller::ControlServer::callback1, &_server, _1, _2));
        thruster2.setCallback(boost::bind(&kraken_controller::ControlServer::callback2, &_server, _1, _2));
        thruster3.setCallback(boost::bind(&kraken_controller::ControlServer::callback3, &_server, _1, _2));
        thruster4.setCallback(boost::bind(&kraken_controller::ControlServer::callback4, &_server, _1, _2));
        thruster5.setCallback(boost::bind(&kraken_controller::ControlServer::callback5, &_server, _1, _2));
        _server.setServers(&_ser1,&_ser2);
        ros::spin();
    }
    else
    {
        std::cerr<<"server 'freq' 'file1' 'file2' .... " <<std::endl;
    }

    return 0;
}
