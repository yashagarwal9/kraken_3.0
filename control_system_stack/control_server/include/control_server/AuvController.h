#ifndef AUVCONTROLLER_H
#define AUVCONTROLLER_H

#include <controller_basic/StateController.h>
#include <pose_server/KrakenPose.h>
#include <kraken_msgs/thrusterData6Thruster.h>
#include <kraken_msgs/thrusterData4Thruster.h>
#include <control_server/paramsConfig.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
namespace kraken_controller
{

class AuvController: public StateController
{
public:
    AuvController();
    virtual ~AuvController();
    void updateState();
    void updateIPState();
    void doControlIteration(const kraken_msgs::krakenPose feedback);
    void setSetPoint(const kraken_msgs::krakenPose &setpoint);
    void getState(kraken_msgs::krakenPose &state);
    void moveForward();
    void moveBack();
    void pause();
    void moveTest();
    //void local2global(kraken_msgs::krakenPose &local);
    void local2global(kraken_msgs::krakenPose &local, kraken_msgs::krakenPose &global);
    void loadParams(const std::vector<std::string> &filenames);
    bool checkError(const kraken_msgs::krakenPose &msg);
    void local2globalAll(kraken_msgs::krakenPose &, kraken_msgs::krakenPose &);
    void changeParams(control_server::paramsConfig &msg, int Thruster_selection);
    std::string curr_file;
    std::vector<ControlParameters*> _control_parameters;
    std::map<std::string,int> _control_parameters_index;
private:

    void multiply(float matrix[][3], float* src_vec, float* dst_vec);
    inline float getYaw(float now,float dest)
    {
        float x=now,y=dest;

        if(y-x>360-y+x)
        {
            return +y-x;
        }
        else
        {
            return -(360-y+x);
        }
    }

protected:
};
}


#endif // AUVCONTROLLER_H
