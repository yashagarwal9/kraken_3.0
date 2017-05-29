#include <control_server/AuvController.h>

namespace kraken_controller
{
AuvController::AuvController():StateController()
{
    for(int i=0; i<21; i++)
    {
        _setPoint[i]=0;
        _error[i]=0;
    }
}

AuvController::~AuvController()
{

}

void AuvController::pause()
{
    std::cerr<<"Pause"<<std::endl;
    ControlParameters *param = _control_parameters[_control_parameters_index["pause"]];
    AuvController::curr_file = "Pause";
    std::cout<<curr_file;
    resetController(param->getGain(), param->getOffset());
}

void AuvController::moveTest()
{
    std::cerr<<"Test Movement"<<std::endl;
    ControlParameters *param = _control_parameters[_control_parameters_index["test"]];
    curr_file = "Test";
    resetController(param->getGain(), param->getOffset());
}

void AuvController::moveBack()
{
    std::cerr<<"Moving back"<<std::endl;
    ControlParameters *param = _control_parameters[_control_parameters_index["moveback"]];
    curr_file = "MoveBack";
    resetController(param->getGain(), param->getOffset());
}

void AuvController::moveForward()
{
    std::cerr<<"Moving forward"<<std::endl;
    ControlParameters *param = _control_parameters[_control_parameters_index["moveforward"]];
    curr_file = "MoveForward";
    resetController(param->getGain(), param->getOffset());
}

void AuvController::setSetPoint(const kraken_msgs::krakenPose &setpoint)
{
    _setPoint[0] = setpoint.data[kraken_core::_px];
    _setPoint[1] = setpoint.data[kraken_core::_py];
    _setPoint[2] = setpoint.data[kraken_core::_pz];

    _setPoint[kraken_core::_px+3] = setpoint.data[kraken_core::_px];
    _setPoint[kraken_core::_py+3] = setpoint.data[kraken_core::_py];
    _setPoint[kraken_core::_pz+3] = setpoint.data[kraken_core::_pz];

    _setPoint[kraken_core::_vx+3] = setpoint.data[kraken_core::_vx];
    _setPoint[kraken_core::_vy+3] = setpoint.data[kraken_core::_vy];
    _setPoint[kraken_core::_vz+3] = setpoint.data[kraken_core::_vz];

    _setPoint[kraken_core::_ax+3] = setpoint.data[kraken_core::_ax];
    _setPoint[kraken_core::_ay+3] = setpoint.data[kraken_core::_ay];
    _setPoint[kraken_core::_az+3] = setpoint.data[kraken_core::_az];


    _setPoint[kraken_core::_roll+3] = setpoint.data[kraken_core::_roll];
    _setPoint[kraken_core::_pitch+3] = setpoint.data[kraken_core::_pitch];
    _setPoint[kraken_core::_yaw+3] = getYaw(_setPoint[kraken_core::_yaw+3],
                                            setpoint.data[kraken_core::_yaw]);

    _setPoint[kraken_core::_roll+6] = setpoint.data[kraken_core::_roll];
    _setPoint[kraken_core::_pitch+6] = setpoint.data[kraken_core::_pitch];
    _setPoint[kraken_core::_yaw+6] = getYaw(_setPoint[kraken_core::_yaw+6],
                                            setpoint.data[kraken_core::_yaw]);

    _setPoint[kraken_core::_w_roll+6] = setpoint.data[kraken_core::_w_roll];
    _setPoint[kraken_core::_w_pitch+6] = setpoint.data[kraken_core::_w_pitch];
    _setPoint[kraken_core::_w_yaw+6] = setpoint.data[kraken_core::_w_yaw];
}

void AuvController::doControlIteration(const kraken_msgs::krakenPose feedback)
{

    for(int i=0; i<kraken_core::countState; i++)
    {
        _feedback.data[i] = feedback.data[i];
    }

    /*float error[kraken_core::_yaw +1 ];
    int multiplier[6] = {1,1,1,1,1,1};

    //new part
    error[kraken_core::_px] = _setPoint[kraken_core::_px] - feedback.data[kraken_core::_px];
    if(fabs(error[kraken_core::_px]) > 0.05) {multiplier[0] = multiplier[1] = (error[kraken_core::_px])/fabs(error[kraken_core::_px]);}
    else { multiplier[0] = multiplier[1] = 0;}

    error[kraken_core::_py] = _setPoint[kraken_core::_py] - feedback.data[kraken_core::_py];
    if(fabs(error[kraken_core::_py]) > 0.05) {multiplier[4] = multiplier[5] = (error[kraken_core::_py])/fabs(error[kraken_core::_py]);}
    else { multiplier[4] = multiplier[5] = 0;}

    error[kraken_core::_pz] = _setPoint[kraken_core::_pz] - feedback.data[kraken_core::_pz];
    if(fabs(error[kraken_core::_pz]) > 0.05) {multiplier[2] = multiplier[3] = (error[kraken_core::_pz])/fabs(error[kraken_core::_pz]);}
    else { multiplier[2] = multiplier[3] = 0;}

    error[kraken_core::_roll] = _setPoint[kraken_core::_roll] - feedback.data[kraken_core::_roll];
    error[kraken_core::_pitch] = _setPoint[kraken_core::_pitch] - feedback.data[kraken_core::_pitch];
    error[kraken_core::_yaw] = _setPoint[kraken_core::_yaw] - feedback.data[kraken_core::_yaw];*/
    //end of new part

    for(int row = 0; row < 6; row++)
    {
        _thruster_data6.data[row] =  _offset[row];// * multiplier[row];

        for(int col = 0; col<21; col++)
        {

            _thruster_data6.data[row] +=  _gain[row][col] * _error[col];
            //ROS_INFO("%lf %d ", _error[col] ,col);

        }
    }

    _thruster_data4.data[0] =  _thruster_data6.data[0];
    _thruster_data4.data[1] =  _thruster_data6.data[1];
    _thruster_data4.data[2] =  _thruster_data6.data[4];
    _thruster_data4.data[3] =  _thruster_data6.data[5];
}


void AuvController::updateIPState()
{

    _error[kraken_core::_px] += _feedback.data[kraken_core::_px];
    _error[kraken_core::_py] += _feedback.data[kraken_core::_py];
    _error[kraken_core::_pz] += _feedback.data[kraken_core::_pz];

    _error[kraken_core::_px+3] = _feedback.data[kraken_core::_px];
    _error[kraken_core::_py+3] = _feedback.data[kraken_core::_py];
    _error[kraken_core::_pz+3] = _feedback.data[kraken_core::_pz];

    _error[kraken_core::_vx+3] = _feedback.data[kraken_core::_vx];
    _error[kraken_core::_vy+3] = _feedback.data[kraken_core::_vy];
    _error[kraken_core::_vz+3] = _feedback.data[kraken_core::_vz];

    _error[kraken_core::_ax+3] = _feedback.data[kraken_core::_ax];
    _error[kraken_core::_ay+3] = _feedback.data[kraken_core::_ay];
    _error[kraken_core::_az+3] = _feedback.data[kraken_core::_az];

    _error[kraken_core::_roll+3] += _setPoint[kraken_core::_roll+3];// - _feedback.data[kraken_core::_roll];
    _error[kraken_core::_pitch+3] += _setPoint[kraken_core::_pitch+3];// - _feedback.data[kraken_core::_pitch];
    _error[kraken_core::_yaw+3] += _setPoint[kraken_core::_yaw+3];// - _feedback.data[kraken_core::_yaw];

    _error[kraken_core::_roll+6] =  _feedback.data[kraken_core::_roll];
    _error[kraken_core::_pitch+6] = _feedback.data[kraken_core::_pitch];
    _error[kraken_core::_yaw+6] = _feedback.data[kraken_core::_yaw];

    _error[kraken_core::_w_roll+6] =  _feedback.data[kraken_core::_w_roll];
    _error[kraken_core::_w_pitch+6] = _feedback.data[kraken_core::_w_pitch];
    _error[kraken_core::_w_yaw+6] =   _feedback.data[kraken_core::_w_yaw];
}

void AuvController::updateState()
{
    float _cos_rpy[3];
    float _sin_rpy[3];


    _cos_rpy[0]   = cos(_feedback.data[kraken_core::_roll]);
    _sin_rpy[0] = sin(_feedback.data[kraken_core::_roll]);

    _cos_rpy[1]   = cos(_feedback.data[kraken_core::_pitch]);
    _sin_rpy[1] = sin(_feedback.data[kraken_core::_pitch]);

    _cos_rpy[2]   = cos(_feedback.data[kraken_core::_yaw]);
    _sin_rpy[2] = sin(_feedback.data[kraken_core::_yaw]);


    //////////////////////////////////////////////////////////////
    float _body_to_world_matrix[3][3];
    _body_to_world_matrix[0][0]   = _cos_rpy[2]*_cos_rpy[1];
    _body_to_world_matrix[1][0]   = _cos_rpy[2]*_sin_rpy[1]*_sin_rpy[0]-_sin_rpy[2]*_cos_rpy[0];
    _body_to_world_matrix[2][0]   = _cos_rpy[2]*_sin_rpy[1]*_cos_rpy[0]+_sin_rpy[2]*_sin_rpy[0];
    _body_to_world_matrix[0][1]   = _sin_rpy[2]*_cos_rpy[1];
    _body_to_world_matrix[1][1]   = _sin_rpy[0]*_sin_rpy[1]*_sin_rpy[2]+_cos_rpy[2]*_cos_rpy[0];
    _body_to_world_matrix[2][1]   = _cos_rpy[0]*_sin_rpy[1]*_sin_rpy[2]-_cos_rpy[2]*_sin_rpy[0];
    _body_to_world_matrix[0][2]   = -_sin_rpy[1];
    _body_to_world_matrix[1][2]   = _sin_rpy[0]*_cos_rpy[1];
    _body_to_world_matrix[2][2]   = _cos_rpy[0]*_cos_rpy[1];


    float _error1[21];
    /*_error1[0] = 0;//_setPoint[0];// - _feedback.data[0];
    _error1[1] = 0;//_setPoint[1];// - _feedback.data[1];
    _error1[2] = 0;//_setPoint[2];// - _feedback.data[2];

    _error1[kraken_core::_px+3] = _setPoint[kraken_core::_px+3] - _feedback.data[kraken_core::_px];
    _error1[kraken_core::_py+3] = _setPoint[kraken_core::_py+3] - _feedback.data[kraken_core::_py];
    _error1[kraken_core::_pz+3] = _setPoint[kraken_core::_pz+3] - _feedback.data[kraken_core::_pz];

    _error1[kraken_core::_vx+3] = _setPoint[kraken_core::_vx+3] - _feedback.data[kraken_core::_vx];
    _error1[kraken_core::_vy+3] = _setPoint[kraken_core::_vy+3] - _feedback.data[kraken_core::_vy];
    _error1[kraken_core::_vz+3] = _setPoint[kraken_core::_vz+3] - _feedback.data[kraken_core::_vz];

    _error1[kraken_core::_ax+3] = 0;//_setPoint[kraken_core::_ax+3] - _feedback.data[kraken_core::_ax];
    _error1[kraken_core::_ay+3] = 0;//_setPoint[kraken_core::_ay+3] - _feedback.data[kraken_core::_ay];
    _error1[kraken_core::_az+3] = 0;//_setPoint[kraken_core::_az+3] - _feedback.data[kraken_core::_az];*/
    _error[0] += _setPoint[0] - _feedback.data[0];
    _error[1] += _setPoint[1] - _feedback.data[1];
    _error[2] += _setPoint[2] - _feedback.data[2];

    _error[kraken_core::_px+3] = _setPoint[kraken_core::_px+3] - _feedback.data[kraken_core::_px];
    _error[kraken_core::_py+3] = _setPoint[kraken_core::_py+3] - _feedback.data[kraken_core::_py];
    _error[kraken_core::_pz+3] = _setPoint[kraken_core::_pz+3] - _feedback.data[kraken_core::_pz];

    _error1[kraken_core::_vx+3] = _setPoint[kraken_core::_vx+3] - _feedback.data[kraken_core::_vx];
    _error1[kraken_core::_vy+3] = _setPoint[kraken_core::_vy+3] - _feedback.data[kraken_core::_vy];
    _error1[kraken_core::_vz+3] = _setPoint[kraken_core::_vz+3] - _feedback.data[kraken_core::_vz];

    _error1[kraken_core::_ax+3] = _setPoint[kraken_core::_ax+3] - _feedback.data[kraken_core::_ax];
    _error1[kraken_core::_ay+3] = _setPoint[kraken_core::_ay+3] - _feedback.data[kraken_core::_ay];
    _error1[kraken_core::_az+3] = _setPoint[kraken_core::_az+3] - _feedback.data[kraken_core::_az];

    /*multiply(_body_to_world_matrix,&_error1[kraken_core::_px+3],&_error[kraken_core::_px+3]);*/
    multiply(_body_to_world_matrix, &_error1[kraken_core::_vx+3], &_error[kraken_core::_vx+3]);
    multiply(_body_to_world_matrix, &_error1[kraken_core::_ax+3], &_error[kraken_core::_ax+3]);


    _error[kraken_core::_roll+3] += _setPoint[kraken_core::_roll+3] - _feedback.data[kraken_core::_roll];
    _error[kraken_core::_pitch+3] += _setPoint[kraken_core::_pitch+3] - _feedback.data[kraken_core::_pitch];
    _error[kraken_core::_yaw+3] += _setPoint[kraken_core::_yaw+3] - _feedback.data[kraken_core::_yaw];

    _error[kraken_core::_roll+6] = _setPoint[kraken_core::_roll+6] - _feedback.data[kraken_core::_roll];
    _error[kraken_core::_pitch+6] = _setPoint[kraken_core::_pitch+6] - _feedback.data[kraken_core::_pitch];
    _error[kraken_core::_yaw+6] = _setPoint[kraken_core::_yaw+6] - _feedback.data[kraken_core::_yaw];

    _error[kraken_core::_w_roll+6] = _setPoint[kraken_core::_w_roll+6] - _feedback.data[kraken_core::_w_roll];
    _error[kraken_core::_w_pitch+6] = _setPoint[kraken_core::_w_pitch+6] - _feedback.data[kraken_core::_w_pitch];
    _error[kraken_core::_w_yaw+6] = _setPoint[kraken_core::_w_yaw+6] - _feedback.data[kraken_core::_w_yaw];
}

void AuvController::loadParams(const std::vector<std::string> &filenames)
{
    for(unsigned int i=0; i<filenames.size(); i++)
    {
        ControlParameters *param = new ControlParameters();
        param->load(filenames[i]);
        _control_parameters.push_back(param);
        _control_parameters_index[param->getName()] = i;
        param->write(std::cerr);
    }
}

bool AuvController::checkError(const kraken_msgs::krakenPose &msg)
{
    float curState[kraken_core::countState + 1];
    float error[kraken_core::countState + 1];
    float thresh[kraken_core::countState + 1];

    bool result;

    thresh[kraken_core::_px] = 0.05;
    thresh[kraken_core::_py] = 0.05;
    thresh[kraken_core::_pz] = 0.05;
    thresh[kraken_core::_roll] = 0.05;
    thresh[kraken_core::_pitch] = 0.05;
    thresh[kraken_core::_yaw] = 0.05;

    curState[kraken_core::_px] = msg.data[kraken_core::_px];
    curState[kraken_core::_py] = msg.data[kraken_core::_py];
    curState[kraken_core::_pz] = msg.data[kraken_core::_pz];
    curState[kraken_core::_roll] = msg.data[kraken_core::_roll];
    curState[kraken_core::_pitch] = msg.data[kraken_core::_pitch];
    curState[kraken_core::_yaw] = msg.data[kraken_core::_yaw];


    error[kraken_core::_px] = fabs(curState[kraken_core::_px] - _setPoint[kraken_core::_px+3]);
    error[kraken_core::_py] = fabs(curState[kraken_core::_py] - _setPoint[kraken_core::_py+3]);
    error[kraken_core::_pz] = fabs(curState[kraken_core::_pz] - _setPoint[kraken_core::_pz+3]);
    error[kraken_core::_roll] = fabs(curState[kraken_core::_roll] - _setPoint[kraken_core::_roll+6]);
    error[kraken_core::_pitch] = fabs(curState[kraken_core::_pitch] - _setPoint[kraken_core::_pitch+6]);
    error[kraken_core::_yaw] = fabs(curState[kraken_core::_yaw] - _setPoint[kraken_core::_yaw+6]);

    result =((error[kraken_core::_px] < thresh[kraken_core::_px ]) && (error[kraken_core::_py] < thresh[kraken_core::_py]) && (error[kraken_core::_pz] < thresh[kraken_core::_pz]) && (error[kraken_core::_roll] < thresh[kraken_core::_roll ]) && (error[kraken_core::_pitch] < thresh[kraken_core::_pitch]) && (error[kraken_core::_yaw] < thresh[kraken_core::_yaw ]));

    /*std::cout<<"error 1 : "<< (error[kraken_core::_px])/fabs(error[kraken_core::_px]) <<std::endl;
    std::cout<<"error 2 : "<< (error[kraken_core::_py])/fabs(error[kraken_core::_py]) <<std::endl;
    std::cout<<"error 3 : "<< (error[kraken_core::_pz])/fabs(error[kraken_core::_pz]) <<std::endl;
    std::cout<<"error 4 : "<< (error[kraken_core::_roll])/fabs(error[kraken_core::_yaw]) <<std::endl;
    std::cout<<"error 5 : "<< (error[kraken_core::_pitch])/fabs(error[kraken_core::_pitch]) <<std::endl;
    std::cout<<"error 6 : "<< (error[kraken_core::_yaw])/fabs(error[kraken_core::_yaw]) <<std::endl;*/

    //std::cout<<" "<<curState[kraken_core::_yaw]<<" : "<<_setPoint[kraken_core::_yaw+6]<<" : "<<error[kraken_core::_yaw] <<std::endl;
    //std::cout<<"result : "<<result<<std::endl;
    //std::cout<<"kd : "<<_gain[0][5]<<std::endl;
    return result;
}

void AuvController::getState(kraken_msgs::krakenPose &state)
{
    state.data[kraken_core::_px] = _setPoint[kraken_core::_px+3];
    state.data[kraken_core::_py] = _setPoint[kraken_core::_py+3];
    state.data[kraken_core::_pz] = _setPoint[kraken_core::_pz+3];

    state.data[kraken_core::_vx] = _setPoint[kraken_core::_vx+3];
    state.data[kraken_core::_vy] = _setPoint[kraken_core::_vy+3];
    state.data[kraken_core::_vz] = _setPoint[kraken_core::_vz+3];

    state.data[kraken_core::_ax] = _setPoint[kraken_core::_ax+3];
    state.data[kraken_core::_ay] =  _setPoint[kraken_core::_ay+3];
    state.data[kraken_core::_az] = _setPoint[kraken_core::_az+3];

    state.data[kraken_core::_roll] = _setPoint[kraken_core::_roll+6];
    state.data[kraken_core::_pitch] = _setPoint[kraken_core::_pitch+6];
    state.data[kraken_core::_yaw] = _setPoint[kraken_core::_yaw+6];

    state.data[kraken_core::_w_roll] = _setPoint[kraken_core::_w_roll+6];
    state.data[kraken_core::_w_pitch] = _setPoint[kraken_core::_w_pitch+6];
    state.data[kraken_core::_w_yaw] = _setPoint[kraken_core::_w_yaw+6];
}


void AuvController::local2globalAll(kraken_msgs::krakenPose &local, kraken_msgs::krakenPose &global)
{
    float _cos_rpy[3];
    float _sin_rpy[3];


    _cos_rpy[0]   = cos(local.data[kraken_core::_roll]);
    _sin_rpy[0]   = sin(local.data[kraken_core::_roll]);

    _cos_rpy[1]   = cos(local.data[kraken_core::_pitch]);
    _sin_rpy[1]   = sin(local.data[kraken_core::_pitch]);

    _cos_rpy[2]   = cos(local.data[kraken_core::_yaw]);
    _sin_rpy[2]   = sin(local.data[kraken_core::_yaw]);


    //////////////////////////////////////////////////////////////
    float _body_to_world_matrix[3][3];
    _body_to_world_matrix[0][0]   = _cos_rpy[2]*_cos_rpy[1];
    _body_to_world_matrix[0][1]   = _cos_rpy[2]*_sin_rpy[1]*_sin_rpy[0]-_sin_rpy[2]*_cos_rpy[0];
    _body_to_world_matrix[0][2]   = _cos_rpy[2]*_sin_rpy[1]*_cos_rpy[0]+_sin_rpy[2]*_sin_rpy[0];
    _body_to_world_matrix[1][0]   = _sin_rpy[2]*_cos_rpy[1];
    _body_to_world_matrix[1][1]   = _sin_rpy[0]*_sin_rpy[1]*_sin_rpy[2]+_cos_rpy[2]*_cos_rpy[0];
    _body_to_world_matrix[1][2]   = _cos_rpy[0]*_sin_rpy[1]*_sin_rpy[2]-_cos_rpy[2]*_sin_rpy[0];
    _body_to_world_matrix[2][0]   = -_sin_rpy[1];
    _body_to_world_matrix[2][1]   = _sin_rpy[0]*_cos_rpy[1];
    _body_to_world_matrix[2][2]   = _cos_rpy[0]*_cos_rpy[1];


    float _local[kraken_core::countState];
    std::cout<<"global x "<<global.data[kraken_core::_px]<<" : "<<global.data[kraken_core::_px]<<std::endl;
    std::cout<<"global y "<<global.data[kraken_core::_py]<<" : "<<global.data[kraken_core::_py]<<std::endl;
    std::cout<<"global z "<<global.data[kraken_core::_pz]<<" : "<<global.data[kraken_core::_pz]<<std::endl;
    _local[kraken_core::_px] = local.data[kraken_core::_px];// - _feedback.data[0];
    _local[kraken_core::_py] = local.data[kraken_core::_py];// - _feedback.data[1];
    _local[kraken_core::_pz] = local.data[kraken_core::_pz];// - _feedback.data[2];

    float _global[kraken_core::countState];
    multiply(_body_to_world_matrix,&_local[kraken_core::_px],&_global[kraken_core::_px]);
    multiply(_body_to_world_matrix,&_local[kraken_core::_vx+3],&_error[kraken_core::_vx+3]);
    multiply(_body_to_world_matrix,&_local[kraken_core::_ax+3],&_error[kraken_core::_ax+3]);

    global.data[kraken_core::_px] = _global[kraken_core::_px];/*+= _global[kraken_core::_px];*/
    global.data[kraken_core::_py] = _global[kraken_core::_py];/*+= _global[kraken_core::_py];*/
    global.data[kraken_core::_pz] = _global[kraken_core::_pz];/*+= _global[kraken_core::_pz];*/

    global.data[kraken_core::_vx] = _global[kraken_core::_vx];/*+= _global[kraken_core::_vx];*/
    global.data[kraken_core::_vy] = _global[kraken_core::_vy];/*+= _global[kraken_core::_vy];*/
    global.data[kraken_core::_vz] = _global[kraken_core::_vz];/*+= _global[kraken_core::_vz];*/

    global.data[kraken_core::_ax] = _global[kraken_core::_ax];/*+= _global[kraken_core::_ax];*/
    global.data[kraken_core::_ay] = _global[kraken_core::_ay];/*+= _global[kraken_core::_ay];*/
    global.data[kraken_core::_az] = _global[kraken_core::_az];/*+= _global[kraken_core::_az];*/


    std::cout<<"global x "<<_global[kraken_core::_px]<<" : "<<global.data[kraken_core::_px]<<std::endl;
    std::cout<<"global y "<<_global[kraken_core::_py]<<" : "<<global.data[kraken_core::_py]<<std::endl;
    std::cout<<"global z "<<_global[kraken_core::_pz]<<" : "<<global.data[kraken_core::_pz]<<std::endl;

    std::cout<<"global vx "<<_global[kraken_core::_vx]<<" : "<<global.data[kraken_core::_vx]<<std::endl;
    std::cout<<"global vy "<<_global[kraken_core::_vy]<<" : "<<global.data[kraken_core::_vy]<<std::endl;
    std::cout<<"global vz "<<_global[kraken_core::_vz]<<" : "<<global.data[kraken_core::_vz]<<std::endl;

    std::cout<<"global ax "<<_global[kraken_core::_ax]<<" : "<<global.data[kraken_core::_ax]<<std::endl;
    std::cout<<"global ay "<<_global[kraken_core::_ay]<<" : "<<global.data[kraken_core::_ay]<<std::endl;
    std::cout<<"global az "<<_global[kraken_core::_az]<<" : "<<global.data[kraken_core::_az]<<std::endl;


    global.data[kraken_core::_roll] = local.data[kraken_core::_roll];
    global.data[kraken_core::_pitch] = local.data[kraken_core::_pitch];
    global.data[kraken_core::_yaw] = local.data[kraken_core::_yaw];

}

void AuvController::local2global(kraken_msgs::krakenPose &local, kraken_msgs::krakenPose &global)
{
    float _cos_rpy[3];
    float _sin_rpy[3];


    _cos_rpy[0]   = cos(local.data[kraken_core::_roll]);
    _sin_rpy[0]   = sin(local.data[kraken_core::_roll]);

    _cos_rpy[1]   = cos(local.data[kraken_core::_pitch]);
    _sin_rpy[1]   = sin(local.data[kraken_core::_pitch]);

    _cos_rpy[2]   = cos(local.data[kraken_core::_yaw]);
    _sin_rpy[2]   = sin(local.data[kraken_core::_yaw]);


    //////////////////////////////////////////////////////////////
    float _body_to_world_matrix[3][3];
    _body_to_world_matrix[0][0]   = _cos_rpy[2]*_cos_rpy[1];
    _body_to_world_matrix[0][1]   = _cos_rpy[2]*_sin_rpy[1]*_sin_rpy[0]-_sin_rpy[2]*_cos_rpy[0];
    _body_to_world_matrix[0][2]   = _cos_rpy[2]*_sin_rpy[1]*_cos_rpy[0]+_sin_rpy[2]*_sin_rpy[0];
    _body_to_world_matrix[1][0]   = _sin_rpy[2]*_cos_rpy[1];
    _body_to_world_matrix[1][1]   = _sin_rpy[0]*_sin_rpy[1]*_sin_rpy[2]+_cos_rpy[2]*_cos_rpy[0];
    _body_to_world_matrix[1][2]   = _cos_rpy[0]*_sin_rpy[1]*_sin_rpy[2]-_cos_rpy[2]*_sin_rpy[0];
    _body_to_world_matrix[2][0]   = -_sin_rpy[1];
    _body_to_world_matrix[2][1]   = _sin_rpy[0]*_cos_rpy[1];
    _body_to_world_matrix[2][2]   = _cos_rpy[0]*_cos_rpy[1];


    float _local[kraken_core::countState];
    std::cout<<"global x "<<global.data[kraken_core::_px]<<" : "<<global.data[kraken_core::_px]<<std::endl;
    std::cout<<"global y "<<global.data[kraken_core::_py]<<" : "<<global.data[kraken_core::_py]<<std::endl;
    std::cout<<"global z "<<global.data[kraken_core::_pz]<<" : "<<global.data[kraken_core::_pz]<<std::endl;
    _local[kraken_core::_px] = local.data[kraken_core::_px];// - _feedback.data[0];
    _local[kraken_core::_py] = local.data[kraken_core::_py];// - _feedback.data[1];
    _local[kraken_core::_pz] = local.data[kraken_core::_pz];// - _feedback.data[2];

    float _global[kraken_core::countState];
    multiply(_body_to_world_matrix,&_local[kraken_core::_px],&_global[kraken_core::_px]);
    //multiply(_body_to_world_matrix,&_local[kraken_core::_vx+3],&_error[kraken_core::_vx+3]);
    //multiply(_body_to_world_matrix,&_local[kraken_core::_ax+3],&_error[kraken_core::_ax+3]);

    global.data[kraken_core::_px] = _global[kraken_core::_px];/*+= _global[kraken_core::_px];*/
    global.data[kraken_core::_py] = _global[kraken_core::_py];/*+= _global[kraken_core::_py];*/
    global.data[kraken_core::_pz] = _global[kraken_core::_pz];/*+= _global[kraken_core::_pz];*/
    /*
     global.data[kraken_core::_vx] += _global[kraken_core::_vx];
     global.data[kraken_core::_vy] += _global[kraken_core::_vy];
     global.data[kraken_core::_vz] += _global[kraken_core::_vz];
     global.data[kraken_core::_ax] += _global[kraken_core::_ax];
     global.data[kraken_core::_ay] += _global[kraken_core::_ay];
     global.data[kraken_core::_az] += _global[kraken_core::_az];
    */

    std::cout<<"global x "<<_global[kraken_core::_px]<<" : "<<global.data[kraken_core::_px]<<std::endl;
    std::cout<<"global y "<<_global[kraken_core::_py]<<" : "<<global.data[kraken_core::_py]<<std::endl;
    std::cout<<"global z "<<_global[kraken_core::_pz]<<" : "<<global.data[kraken_core::_pz]<<std::endl;
    /*
     std::cout<<"global x "<<_global[kraken_core::_vx]<<" : "<<global.data[kraken_core::_vx]<<std::endl;
     std::cout<<"global y "<<_global[kraken_core::_vy]<<" : "<<global.data[kraken_core::_vy]<<std::endl;
     std::cout<<"global z "<<_global[kraken_core::_vz]<<" : "<<global.data[kraken_core::_vz]<<std::endl;
     std::cout<<"global x "<<_global[kraken_core::_ax]<<" : "<<global.data[kraken_core::_ax]<<std::endl;
     std::cout<<"global y "<<_global[kraken_core::_ay]<<" : "<<global.data[kraken_core::_ay]<<std::endl;
     std::cout<<"global z "<<_global[kraken_core::_az]<<" : "<<global.data[kraken_core::_az]<<std::endl;
    */
    /*
    global.data[kraken_core::_roll] = local.data[kraken_core::_roll];
    global.data[kraken_core::_pitch] = local.data[kraken_core::_pitch];
    global.data[kraken_core::_yaw] = local.data[kraken_core::_yaw];
    */

}

void AuvController::multiply(float matrix[][3], float* src_vec, float* dst_vec)
{
    for(int i=0; i<3; i++)
    {
        float val=0;

        for(int j =0; j<3; j++)
        {
            val+=matrix[i][j]*src_vec[j];
        }

        dst_vec[i] = val;
    }
}
void AuvController::changeParams(control_server::paramsConfig &msg, int Thruster_selection){
  int n_map = _control_parameters_index[AuvController::curr_file];
    float *offset = _control_parameters[n_map]->getOffset();
    float **gain = _control_parameters[n_map]->getGain();
        if(Thruster_selection == 0){
          offset[0] = msg.offset;
          gain[0][0] = msg.Gain_i_px;
          gain[0][1] = msg.Gain_i_py;
          gain[0][2] = msg.Gain_i_pz;
          gain[0][3] = msg.Gain_px;
          gain[0][4] = msg.Gain_py;
          gain[0][5] = msg.Gain_pz;
          gain[0][6] = msg.Gain_vx;
          gain[0][7] = msg.Gain_vy;
          gain[0][8] = msg.Gain_vz;
          gain[0][9] = msg.Gain_ax;
          gain[0][10] = msg.Gain_ay;
          gain[0][11] = msg.Gain_az;
          gain[0][12] = msg.Gain_i_roll;
          gain[0][13] = msg.Gain_i_pitch;
          gain[0][14] = msg.Gain_i_yaw;
          gain[0][15] = msg.Gain_roll;
          gain[0][16] = msg.Gain_pitch;
          gain[0][17] = msg.Gain_yaw;
          gain[0][18] = msg.Gain_a_roll;
          gain[0][19] = msg.Gain_a_pitch;
          gain[0][20] = msg.Gain_a_yaw;
        }
        if(Thruster_selection == 1){
          offset[1] = msg.offset;
          gain[1][0] = msg.Gain_i_px;
          gain[1][1] = msg.Gain_i_py;
          gain[1][2] = msg.Gain_i_pz;
          gain[1][3] = msg.Gain_px;
          gain[1][4] = msg.Gain_py;
          gain[1][5] = msg.Gain_pz;
          gain[1][6] = msg.Gain_vx;
          gain[1][7] = msg.Gain_vy;
          gain[1][8] = msg.Gain_vz;
          gain[1][9] = msg.Gain_ax;
          gain[1][10] = msg.Gain_ay;
          gain[1][11] = msg.Gain_az;
          gain[1][12] = msg.Gain_i_roll;
          gain[1][13] = msg.Gain_i_pitch;
          gain[1][14] = msg.Gain_i_yaw;
          gain[1][15] = msg.Gain_roll;
          gain[1][16] = msg.Gain_pitch;
          gain[1][17] = msg.Gain_yaw;
          gain[1][18] = msg.Gain_a_roll;
          gain[1][19] = msg.Gain_a_pitch;
          gain[1][20] = msg.Gain_a_yaw;
        }
        if(Thruster_selection == 2){
          offset[2] = msg.offset;
          gain[2][0] = msg.Gain_i_px;
          gain[2][1] = msg.Gain_i_py;
          gain[2][2] = msg.Gain_i_pz;
          gain[2][3] = msg.Gain_px;
          gain[2][4] = msg.Gain_py;
          gain[2][5] = msg.Gain_pz;
          gain[2][6] = msg.Gain_vx;
          gain[2][7] = msg.Gain_vy;
          gain[2][8] = msg.Gain_vz;
          gain[2][9] = msg.Gain_ax;
          gain[2][10] = msg.Gain_ay;
          gain[2][11] = msg.Gain_az;
          gain[2][12] = msg.Gain_i_roll;
          gain[2][13] = msg.Gain_i_pitch;
          gain[2][14] = msg.Gain_i_yaw;
          gain[2][15] = msg.Gain_roll;
          gain[2][16] = msg.Gain_pitch;
          gain[2][17] = msg.Gain_yaw;
          gain[2][18] = msg.Gain_a_roll;
          gain[2][19] = msg.Gain_a_pitch;
          gain[2][20] = msg.Gain_a_yaw;
        }
        if(Thruster_selection == 3){
          offset[3] = msg.offset;
          gain[3][0] = msg.Gain_i_px;
          gain[3][1] = msg.Gain_i_py;
          gain[3][2] = msg.Gain_i_pz;
          gain[3][3] = msg.Gain_px;
          gain[3][4] = msg.Gain_py;
          gain[3][5] = msg.Gain_pz;
          gain[3][6] = msg.Gain_vx;
          gain[3][7] = msg.Gain_vy;
          gain[3][8] = msg.Gain_vz;
          gain[3][9] = msg.Gain_ax;
          gain[3][10] = msg.Gain_ay;
          gain[3][11] = msg.Gain_az;
          gain[3][12] = msg.Gain_i_roll;
          gain[3][13] = msg.Gain_i_pitch;
          gain[3][14] = msg.Gain_i_yaw;
          gain[3][15] = msg.Gain_roll;
          gain[3][16] = msg.Gain_pitch;
          gain[3][17] = msg.Gain_yaw;
          gain[3][18] = msg.Gain_a_roll;
          gain[3][19] = msg.Gain_a_pitch;
          gain[3][20] = msg.Gain_a_yaw;
        }
        if(Thruster_selection == 4){
          offset[4] = msg.offset;
          gain[4][0] = msg.Gain_i_px;
          gain[4][1] = msg.Gain_i_py;
          gain[4][2] = msg.Gain_i_pz;
          gain[4][3] = msg.Gain_px;
          gain[4][4] = msg.Gain_py;
          gain[4][5] = msg.Gain_pz;
          gain[4][6] = msg.Gain_vx;
          gain[4][7] = msg.Gain_vy;
          gain[4][8] = msg.Gain_vz;
          gain[4][9] = msg.Gain_ax;
          gain[4][10] = msg.Gain_ay;
          gain[4][11] = msg.Gain_az;
          gain[4][12] = msg.Gain_i_roll;
          gain[4][13] = msg.Gain_i_pitch;
          gain[4][14] = msg.Gain_i_yaw;
          gain[4][15] = msg.Gain_roll;
          gain[4][16] = msg.Gain_pitch;
          gain[4][17] = msg.Gain_yaw;
          gain[4][18] = msg.Gain_a_roll;
          gain[4][19] = msg.Gain_a_pitch;
          gain[4][20] = msg.Gain_a_yaw;
        }
        if(Thruster_selection == 5){
          offset[5] = msg.offset;
          gain[5][0] = msg.Gain_i_px;
          gain[5][1] = msg.Gain_i_py;
          gain[5][2] = msg.Gain_i_pz;
          gain[5][3] = msg.Gain_px;
          gain[5][4] = msg.Gain_py;
          gain[5][5] = msg.Gain_pz;
          gain[5][6] = msg.Gain_vx;
          gain[5][7] = msg.Gain_vy;
          gain[5][8] = msg.Gain_vz;
          gain[5][9] = msg.Gain_ax;
          gain[5][10] = msg.Gain_ay;
          gain[5][11] = msg.Gain_az;
          gain[5][12] = msg.Gain_i_roll;
          gain[5][13] = msg.Gain_i_pitch;
          gain[5][14] = msg.Gain_i_yaw;
          gain[5][15] = msg.Gain_roll;
          gain[5][16] = msg.Gain_pitch;
          gain[5][17] = msg.Gain_yaw;
          gain[5][18] = msg.Gain_a_roll;
          gain[5][19] = msg.Gain_a_pitch;
          gain[5][20] = msg.Gain_a_yaw;
        }
        std::string str = "/home/yash/catkin_ws/src/kraken_3.0/control_system_stack/control_server/parameters/";
        std::fstream fp;
        str = str.append(curr_file).c_str();
        fp.open(str.append(".cp").c_str(), std::ios::trunc | std::ios::out);
        if(fp.is_open()){
          _control_parameters[n_map]->write(&fp);
        }
        else ROS_INFO("Unable to open file %s", curr_file.c_str());
    }
}
