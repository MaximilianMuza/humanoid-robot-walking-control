#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller_hw.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"
#include <fstream>
#include <tf/tf.h>

Vars vars;
Params params;
Workspace work;
Settings settings;

namespace dyros_jet_controller
{ 
  ofstream MJ_graph("/home/maximilian/Projects/humanoid-robot-walking-control/homework-9/MJ_graph.txt");

void WalkingController::compute()
{   
  if(walking_enable_ == true)
  {
    updateInitialState();  
    getRobotState();  
    floatToSupportFootstep(); 

    Eigen::Vector2d left_zmp, right_zmp;
    left_zmp(0) = l_ft_(4)/l_ft_(2) + lfoot_trajectory_support_.translation()(0);
    left_zmp(1) = l_ft_(3)/l_ft_(2) + lfoot_trajectory_support_.translation()(1);

    right_zmp(0) = r_ft_(4)/r_ft_(2) + rfoot_trajectory_support_.translation()(0);
    right_zmp(1) = r_ft_(3)/r_ft_(2) + rfoot_trajectory_support_.translation()(1);

    zmp_measured_(0) = (left_zmp(0) * l_ft_(2) + right_zmp(0) * r_ft_(2))/(l_ft_(2) + r_ft_(2));
    zmp_measured_(1) = (left_zmp(1) * l_ft_(2) + right_zmp(1) * r_ft_(2))/(l_ft_(2) + r_ft_(2));

    
    if(ready_for_thread_flag_ == false)
    { ready_for_thread_flag_ = true; }
    
    if(ready_for_compute_flag_ == true)
    {
      if(current_step_num_< total_step_num_)
      {                  
        getZmpTrajectory();
        getCPTrajectory_M(); // HW#8
        getComTrajectory();        
        getPelvTrajectory();     
        getFootTrajectory();   
        supportToFloatPattern();
        computeIkControl_MJ(pelv_trajectory_float_, lfoot_trajectory_float_, rfoot_trajectory_float_, q_des);
          
        for(int i=0; i<12; i++)
        { desired_q_(i) = q_des(i); }
        desired_q_not_compensated_ = desired_q_ ;  
        hip_compensator();
        updateNextStepTime();
      }
      else
      {
        desired_q_ = current_q_;
      }
    }
    else
    {
      desired_q_ = current_q_;
    }
  }
}

void WalkingController::setTarget(int walk_mode, bool hip_compensation, bool lqr, int ik_mode, bool heel_toe,
                                  bool is_right_foot_swing, double x, double y, double z, double height, double theta,
                                  double step_length_x, double step_length_y, bool walking_pattern)
{
  target_x_ = x;
  target_y_ = y;
  target_z_ = z;
  com_height_ = height;
  target_theta_ = theta;
  step_length_x_ = step_length_x;
  step_length_y_ = step_length_y;
  ik_mode_ = ik_mode;
  walk_mode_ = walk_mode;
  hip_compensator_mode_ = hip_compensation; //uint32 compensator_mode[0] : HIP_COMPENSTOR    uint32 compensator_mode[1] : EXTERNAL_ENCODER   
  is_right_foot_swing_ = is_right_foot_swing;  
  walkingPatternDCM_ = walking_pattern; 

  parameterSetting();
}

void WalkingController::setEnable(bool enable)
{
  walking_enable_ = enable;
  desired_q_ = current_q_;
}

void WalkingController::updateControlMask(unsigned int *mask)
{
  if(walking_enable_)
  {
    for (int i=0; i<total_dof_-18; i++) //control only leg
    {
      mask[i] = (mask[i] | PRIORITY);
    }
    mask[total_dof_-1] = (mask[total_dof_-1] & ~PRIORITY); //Gripper
    mask[total_dof_-2] = (mask[total_dof_-2] & ~PRIORITY); //Gripper
    mask[total_dof_-3] = (mask[total_dof_-3] & ~PRIORITY); //Head
    mask[total_dof_-4] = (mask[total_dof_-4] & ~PRIORITY); //Head
  }
  else
  {
    for (int i=0; i<total_dof_; i++)
    {
      mask[i] = (mask[i] & ~PRIORITY);
    }
  }
}

void WalkingController::writeDesired(const unsigned int *mask, VectorQd& desired_q)
{
  for(unsigned int i=0; i<total_dof_; i++)
  {     
    if( mask[i] >= PRIORITY && mask[i] < PRIORITY * 2 )
    {
      desired_q(i) = desired_q_(i);
    }         
  }
}

void WalkingController::parameterSetting()
{
  t_double1_ = 0.10*hz_; 
  t_double2_ = 0.10*hz_;
  t_rest_init_ = 0.05*hz_;
  t_rest_last_ = 0.05*hz_;
  t_total_= 1.2*hz_;
  t_temp_ = 3.0*hz_;
  t_last_ = t_total_ + t_temp_; 
  t_start_ = t_temp_ + 1 ;

  
  t_start_real_ = t_start_ + t_rest_init_;
  
  current_step_num_ = 0;
  walking_tick_ = 0; 
  foot_height_ = 0.05; 
  gyro_frame_flag_ = false;
  com_control_mode_ = true;
  estimator_flag_ = false; 
}

void WalkingController::getRobotState()
{
  Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
  q_temp.setZero();
  qdot_temp;  
  q_temp.segment<28>(6) = current_q_.segment<28>(0);   
  qdot_temp.segment<28>(6)= current_qdot_.segment<28>(0);
 
  model_.updateKinematics(q_temp, qdot_temp);
  com_float_current_ = model_.getCurrentCom();
  com_float_current_dot_= model_.getCurrentComDot();
  lfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)0); 
  rfoot_float_current_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)1);
    
  if(foot_step_(current_step_num_, 6) == 0) 
  { supportfoot_float_current_ = rfoot_float_current_; }
  else if(foot_step_(current_step_num_, 6) == 1)
  { supportfoot_float_current_ = lfoot_float_current_; }

  if(current_step_num_ != 0 && walking_tick_ == t_start_) // step change
  { 
    if(foot_step_(current_step_num_, 6) == 0)
    {
      com_support_current_dot = (DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(lfoot_float_current_), com_float_current_) - com_support_current_)*hz_;
    }
    else if(foot_step_(current_step_num_, 6) == 1)
    {
      com_support_current_dot = (DyrosMath::multiplyIsometry3dVector3d(DyrosMath::inverseIsometry3d(rfoot_float_current_), com_float_current_) - com_support_current_)*hz_;
    }
  }

  pelv_support_current_ = DyrosMath::inverseIsometry3d(supportfoot_float_current_);
  pelv_float_current_.setIdentity();
  lfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,lfoot_float_current_);
  rfoot_support_current_ = DyrosMath::multiplyIsometry3d(pelv_support_current_,rfoot_float_current_);     
 
  com_support_current_b = com_support_current_;
  com_support_current_ =  DyrosMath::multiplyIsometry3dVector3d(pelv_support_current_, com_float_current_);
  if(walking_tick_ != t_start_)
  { com_support_current_dot = (com_support_current_ - com_support_current_b)*hz_; }   
  
  r_ft_ = model_.getRightFootForce();
  l_ft_ = model_.getLeftFootForce();
}

void WalkingController::calculateFootStepTotal()
{
  double initial_rot = 0.0;
  double final_rot = 0.0;
  double initial_drot = 0.0;
  double final_drot = 0.0;

  initial_rot = atan2(target_y_, target_x_);

  if(initial_rot > 0.0)
    initial_drot = 10*DEG2RAD;
  else
    initial_drot = -10*DEG2RAD;

  unsigned int initial_total_step_number = initial_rot/initial_drot;
  double initial_residual_angle = initial_rot - initial_total_step_number*initial_drot;

  final_rot = target_theta_ - initial_rot;
  if(final_rot > 0.0)
    final_drot = 10*DEG2RAD;
  else
    final_drot = -10*DEG2RAD;

  unsigned int final_total_step_number = final_rot/final_drot;
  double final_residual_angle = final_rot - final_total_step_number*final_drot;
  double length_to_target = sqrt(target_x_*target_x_ + target_y_*target_y_);
  double dlength = step_length_x_;
  unsigned int middle_total_step_number = length_to_target/dlength;
  double middle_residual_length = length_to_target - middle_total_step_number*dlength;

  if(length_to_target == 0)
  {
    middle_total_step_number = 5;
    dlength = 0;
  }

  unsigned int number_of_foot_step;

  int del_size;

  del_size = 1;
  number_of_foot_step = initial_total_step_number*del_size + middle_total_step_number*del_size + final_total_step_number*del_size;

  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    if(initial_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(initial_residual_angle)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length)>=0.0001)
  {
    if(middle_total_step_number % 2 == 0)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
    {
      if(abs(middle_residual_length)>= 0.0001)
        number_of_foot_step = number_of_foot_step + 3*del_size;
      else
        number_of_foot_step = number_of_foot_step + del_size;
    }
  }

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    if(abs(final_residual_angle) >= 0.0001)
      number_of_foot_step = number_of_foot_step + 2*del_size;
    else
      number_of_foot_step = number_of_foot_step + del_size;
  }


  foot_step_.resize(number_of_foot_step, 7);
  foot_step_.setZero();
  foot_step_support_frame_.resize(number_of_foot_step, 7);
  foot_step_support_frame_.setZero();

  int index = 0;
  int temp, temp2, temp3, is_right;

  if(is_right_foot_swing_ == true)
    is_right = 1;
  else
    is_right = -1;


  temp = -is_right;
  temp2 = -is_right;
  temp3 = -is_right;


  if(initial_total_step_number != 0 || abs(initial_residual_angle) >= 0.0001)
  {
    for (int i =0 ; i < initial_total_step_number; i++)
    {
      temp *= -1;
      foot_step_(index,0) = temp*0.127794*sin((i+1)*initial_drot);
      foot_step_(index,1) = -temp*0.127794*cos((i+1)*initial_drot);
      foot_step_(index,5) = (i+1)*initial_drot;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index++;
    }

    if(temp == is_right)
    {
      if(abs(initial_residual_angle) >= 0.0001)
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot+initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;

      }
      else
      {
        temp *= -1;

        foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
        foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
        foot_step_(index,6) = 0.5 + 0.5*temp;
        index++;
      }
    }
    else if(temp == -is_right)
    {
      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;

      temp *= -1;

      foot_step_(index,0) = temp*0.127794*sin((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,1) = -temp*0.127794*cos((initial_total_step_number)*initial_drot + initial_residual_angle);
      foot_step_(index,5) = (initial_total_step_number)*initial_drot + initial_residual_angle;
      foot_step_(index,6) = 0.5 + 0.5*temp;
      index ++;
    }
  }

  if(middle_total_step_number != 0 || abs(middle_residual_length) >= 0.0001)
  {
    for (int i = 0 ; i < middle_total_step_number; i++)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(i+1)) + temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(i+1)) - temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index ++;
    }

    if(temp2 == is_right)
    {
      if(abs(middle_residual_length) >= 0.0001)
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;

        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;

        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
      else
      {
        temp2 *= -1;

        foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
        foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
        foot_step_(index,5) = initial_rot;
        foot_step_(index,6) = 0.5 + 0.5*temp2;
        index++;
      }
    }
    else if(temp2 == -is_right)
    {
      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;

      temp2 *= -1;

      foot_step_(index,0) = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) + temp2*sin(initial_rot)*(0.127794);
      foot_step_(index,1) = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length) - temp2*cos(initial_rot)*(0.127794);
      foot_step_(index,5) = initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp2;
      index++;
    }
  }

  double final_position_x = cos(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);
  double final_position_y = sin(initial_rot)*(dlength*(middle_total_step_number) + middle_residual_length);

  if(final_total_step_number != 0 || abs(final_residual_angle) >= 0.0001)
  {
    for(int i = 0 ; i < final_total_step_number; i++)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin((i+1)*final_drot + initial_rot);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos((i+1)*final_drot + initial_rot);
      foot_step_(index,5) = (i+1)*final_drot + initial_rot;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }

    if(abs(final_residual_angle) >= 0.0001)
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;

      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
    else
    {
      temp3 *= -1;

      foot_step_(index,0) = final_position_x + temp3*0.127794*sin(target_theta_);
      foot_step_(index,1) = final_position_y - temp3*0.127794*cos(target_theta_);
      foot_step_(index,5) = target_theta_;
      foot_step_(index,6) = 0.5 + 0.5*temp3;
      index++;
    }
  }
}


void WalkingController::floatToSupportFootstep()
{
  Eigen::Isometry3d reference;

  if(current_step_num_ == 0)
  {
    if(foot_step_(0,6) == 0)
    {
      reference.translation() = rfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
    else
    {
      reference.translation() = lfoot_float_init_.translation();
      reference.translation()(2) = 0.0;
      reference.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(lfoot_float_init_.linear())(2));
      reference.translation()(0) = 0.0;
    }
  }
  else
  {
    reference.linear() = DyrosMath::rotateWithZ(foot_step_(current_step_num_-1,5));
    for(int i=0 ;i<3; i++)
      reference.translation()(i) = foot_step_(current_step_num_-1,i);
  }

  Eigen::Vector3d temp_local_position;
  Eigen::Vector3d temp_global_position;

  for(int i = 0; i < total_step_num_; i++)
  {
    for(int j = 0; j < 3; j ++)
      temp_global_position(j) = foot_step_(i,j);

    temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

    for(int j=0; j<3; j++)
      foot_step_support_frame_(i,j) = temp_local_position(j);

    foot_step_support_frame_(i,3) = foot_step_(i,3);
    foot_step_support_frame_(i,4) = foot_step_(i,4);
    foot_step_support_frame_(i,5) = foot_step_(i,5) - foot_step_(current_step_num_-1,5);

    if(current_step_num_ == 0)
    { foot_step_support_frame_(i,5) = foot_step_(i,5) - supportfoot_float_init_(5); }
  }

  for(int j=0;j<3;j++)
    temp_global_position(j) = supportfoot_float_init_(j);

  temp_local_position = reference.linear().transpose()*(temp_global_position - reference.translation());

  for(int j=0;j<3;j++)
    supportfoot_support_init_(j) = temp_local_position(j);

  supportfoot_support_init_(3) = supportfoot_float_init_(3);
  supportfoot_support_init_(4) = supportfoot_float_init_(4);

  if(current_step_num_ == 0)
    supportfoot_support_init_(5) = 0;
  else
    supportfoot_support_init_(5) = supportfoot_float_init_(5) - foot_step_(current_step_num_-1,5);
}


void WalkingController::updateInitialState()
{
  if(walking_tick_ == 0)
  {
    calculateFootStepTotal(); 
 
    com_float_init_ = model_.getCurrentCom();
    pelv_float_init_.setIdentity();
    lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));    
    
    Eigen::Isometry3d ref_frame;

    if(foot_step_(0, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }    
    else if(foot_step_(0, 6) == 1)
    { ref_frame = lfoot_float_init_; }
    
    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);
    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
    
    com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation();
    
    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear());
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear());

    supportfoot_float_init_.setZero();
    swingfoot_float_init_.setZero();

    if(foot_step_(0,6) == 1) //left suppport foot
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    else
    {
      for(int i=0; i<2; i++)
        supportfoot_float_init_(i) = rfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        supportfoot_float_init_(i+3) = DyrosMath::rot2Euler(rfoot_float_init_.linear())(i);

      for(int i=0; i<2; i++)
        swingfoot_float_init_(i) = lfoot_float_init_.translation()(i);
      for(int i=0; i<3; i++)
        swingfoot_float_init_(i+3) = DyrosMath::rot2Euler(lfoot_float_init_.linear())(i);

      supportfoot_float_init_(0) = 0.0;
      swingfoot_float_init_(0) = 0.0;
    }
    pelv_support_start_ = pelv_support_init_;
    total_step_num_ = foot_step_.col(1).size();
    xi_ = com_support_init_(0); // preview parameter
    yi_ = com_support_init_(1);
    zc_ = com_support_init_(2);     
    
  }
  else if(current_step_num_ != 0 && walking_tick_ == t_start_) // step change 
  {  
    Eigen::Matrix<double, DyrosJetModel::MODEL_WITH_VIRTUAL_DOF, 1> q_temp, qdot_temp;
    q_temp.setZero();
    qdot_temp;
    q_temp.segment<28>(6) = current_q_.segment<28>(0);
    qdot_temp.segment<28>(6)= current_qdot_.segment<28>(0);   
     
    model_.updateKinematics(q_temp, qdot_temp);

    lfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(0));
    rfoot_float_init_ = model_.getCurrentTransform((DyrosJetModel::EndEffector)(1));
    com_float_init_ = model_.getCurrentCom();
    pelv_float_init_.setIdentity();  

    Eigen::Isometry3d ref_frame;

    if(foot_step_(current_step_num_, 6) == 0)  //right foot support
    { ref_frame = rfoot_float_init_; }
    else if(foot_step_(current_step_num_, 6) == 1)
    { ref_frame = lfoot_float_init_; }

    pelv_support_init_ = DyrosMath::inverseIsometry3d(ref_frame);
    com_support_init_ = pelv_support_init_.linear()*com_float_init_ + pelv_support_init_.translation(); 
    pelv_support_euler_init_ = DyrosMath::rot2Euler(pelv_support_init_.linear()); 

    lfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),lfoot_float_init_);
    rfoot_support_init_ = DyrosMath::multiplyIsometry3d(DyrosMath::inverseIsometry3d(ref_frame),rfoot_float_init_);    
    rfoot_support_euler_init_ = DyrosMath::rot2Euler(rfoot_support_init_.linear());
    lfoot_support_euler_init_ = DyrosMath::rot2Euler(lfoot_support_init_.linear()); 
  }  
} 

void WalkingController::getZmpTrajectory()
{
  unsigned int planning_step_number = 3;
  unsigned int norm_size = 0;
  
  if(current_step_num_ >= total_step_num_ - planning_step_number)
    norm_size = (t_last_ - t_start_ + 1)*(total_step_num_ - current_step_num_) + 20*hz_;
  else
    norm_size = (t_last_ - t_start_ + 1)*(planning_step_number); 
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_ + 1;
 
  zmpGenerator(norm_size, planning_step_number);   
}

void WalkingController::zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num)
{ 
  ref_zmp_.resize(norm_size, 2); 
  Eigen::VectorXd temp_px;
  Eigen::VectorXd temp_py;
  
  unsigned int index = 0; 

  if(current_step_num_ == 0)   
  {
    for (int i = 0; i <= t_temp_; i++)  
    {
      if(i < 0.5*hz_) 
      {
        ref_zmp_(i,0) = com_support_init_(0) ;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else if(i < 1.5*hz_) 
      {
        double del_x = i - 0.5*hz_;
        ref_zmp_(i,0) = com_support_init_(0) - del_x * com_support_init_(0)/(1.0*hz_);
        ref_zmp_(i,1) = com_support_init_(1) ;
      }
      else 
      {
        ref_zmp_(i,0) = 0.0;
        ref_zmp_(i,1) = com_support_init_(1) ;
      }      
      index++;
    }    
  }
  if(current_step_num_ >= total_step_num_ - planning_step_num)
  {  
    for(unsigned int i = current_step_num_; i < total_step_num_; i++)
    {
      onestepZmp(i, temp_px, temp_py);
     
      for(unsigned int j = 0; j < t_total_; j++)
      {
        ref_zmp_(index + j, 0) = temp_px(j);
        ref_zmp_(index + j, 1) = temp_py(j);    
      }
      index = index + t_total_;
    }
    
    for(unsigned int j = 0; j < 20*hz_; j++)
    {
      ref_zmp_(index + j, 0) = ref_zmp_(index -1, 0);
      ref_zmp_(index + j, 1) = ref_zmp_(index -1, 1);
    }
    index = index + 20*hz_;      
  }
  else  
  { 
    for(unsigned int i = current_step_num_; i < current_step_num_ + planning_step_num; i++)  
    {
      onestepZmp(i, temp_px, temp_py);
      for (unsigned int j = 0; j < t_total_; j++)  
      {
        ref_zmp_(index+j,0) = temp_px(j);
        ref_zmp_(index+j,1) = temp_py(j);
      }      
      index = index + t_total_;  
    }   
  }   
}


void WalkingController::onestepZmp(unsigned int current_step_number, Eigen::VectorXd& temp_px, Eigen::VectorXd& temp_py)
{
  temp_px.resize(t_total_);  
  temp_py.resize(t_total_);
  temp_px.setZero();
  temp_py.setZero();

  double Kx = 0, Ky = 0, A = 0, B = 0, wn = 0;
  if(current_step_number == 0)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = -(foot_step_support_frame_(current_step_number, 1))/2 ;
    B =  (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45));
        
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      {
        temp_px(i) = 0;
        temp_py(i) = (com_offset_(1) + com_support_init_(1)) + Ky / (t_rest_init_ + t_double1_)* (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_ ) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = supportfoot_support_init_(0);
        temp_py(i) = supportfoot_support_init_(1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_  && i < t_total_) //1.05 ~ 1.15초 , 210 ~ 230 tick 
      {
        temp_px(i) = B - Kx + Kx / (t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (supportfoot_support_init_(1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }    
  }
  else if(current_step_number == 1)
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - supportfoot_support_init_(1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (supportfoot_support_init_(0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45)));
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    
    for(int i = 0; i < t_total_; i++)
    {
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 10 ~ 30 tick
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number-1, 0) + supportfoot_support_init_(0))/2 + Kx / (t_rest_init_+ t_double1_) * (i+1);
        temp_py(i) = (foot_step_support_frame_(current_step_number-1, 1) + supportfoot_support_init_(1))/2 + Ky / (t_rest_init_+ t_double1_) * (i+1);
      }
      else if(i >= t_rest_init_ + t_double1_ && i < t_total_ - t_rest_last_ - t_double2_) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_(current_step_number-1, 0);
        temp_py(i) = foot_step_support_frame_(current_step_number-1, 1);
      }
      else if(i >= t_total_ - t_rest_last_ - t_double2_ && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      {               
        temp_px(i) = (foot_step_support_frame_(current_step_number-1, 0) + foot_step_support_frame_(current_step_number, 0))/2 - Kx + Kx /(t_rest_last_ + t_double2_) * (i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    }
  }
  else
  { 
    wn = sqrt(GRAVITY / com_support_init_(2));
    A = (foot_step_support_frame_(current_step_number-1, 1) - foot_step_support_frame_(current_step_number-2, 1))/2 ;
    B = foot_step_support_frame_(current_step_number-1, 0) - (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
    Kx = (B * 0.15 * wn) / ((0.15*wn) + tanh(wn*(0.45))) ;
    Ky = (A * 0.15 * wn * tanh(wn*0.45))/(1 + 0.15 * wn * tanh(wn*0.45)); 
    for(int i = 0; i < t_total_; i++)
    {      
      if(i >= 0 && i < t_rest_init_ + t_double1_) //0 ~ 0.15초 , 0 ~ 30 tick
      { 
        temp_px(i) = (foot_step_support_frame_(current_step_number-2, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 + Kx/(t_rest_init_ + t_double1_)*(i+1);
        temp_py(i) = (foot_step_support_frame_(current_step_number-2, 1) + foot_step_support_frame_(current_step_number-1, 1))/2 + Ky/(t_rest_init_ + t_double1_)*(i+1);
      }
      else if(i >= (t_rest_init_ + t_double1_) && i < (t_total_ - t_rest_last_ - t_double2_)) //0.15 ~ 1.05초 , 30 ~ 210 tick
      {
        temp_px(i) = foot_step_support_frame_(current_step_number-1, 0) ;
        temp_py(i) = foot_step_support_frame_(current_step_number-1, 1) ;
      }
      else if( i >= (t_total_ - t_rest_last_ - t_double2_) && (i < t_total_) && (current_step_num_ == total_step_num_ - 1))
      {
        temp_px(i) = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number-1, 0))/2;
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }       
      else if(i >= (t_total_ - t_rest_last_ - t_double2_) && i < t_total_) //1.05 ~ 1.2초 , 210 ~ 240 tick 
      { 
        temp_px(i) = (foot_step_support_frame_(current_step_number, 0) + foot_step_support_frame_(current_step_number-1, 0))/2 -Kx + Kx/(t_rest_last_ + t_double2_)*(i+1 - (t_total_ - t_rest_last_ - t_double2_));
        temp_py(i) = Ky + (foot_step_support_frame_(current_step_number-1, 1) + foot_step_support_frame_(current_step_number, 1))/2 + Ky/(t_rest_last_ + t_double2_)*-(i+1 - (t_total_ - t_rest_last_ - t_double2_));
      }
    } 
  }  
}


void WalkingController::preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C)
{
  A.resize(3,3);
  A(0,0) = 1.0;
  A(0,1) = dt;
  A(0,2) = dt*dt*0.5;
  A(1,0) = 0;
  A(1,1) = 1.0;
  A(1,2) = dt;
  A(2,0) = 0;
  A(2,1) = 0;
  A(2,2) = 1;
  
  B.resize(3);
  B(0) = dt*dt*dt/6;
  B(1) = dt*dt/2;
  B(2) = dt;
  
  C.resize(1,3);
  C(0,0) = 1;
  C(0,1) = 0;
  C(0,2) = -zc_/GRAVITY;

  Eigen::MatrixXd A_bar;
  Eigen::VectorXd B_bar;

  B_bar.resize(4);    
  B_bar.segment(0,1) = C*B; 
  B_bar.segment(1,3) = B;
  
  Eigen::Matrix1x4d B_bar_tran;
  B_bar_tran = B_bar.transpose();
  
  Eigen::MatrixXd I_bar;
  Eigen::MatrixXd F_bar;
  A_bar.resize(4,4);
  I_bar.resize(4,1);
  F_bar.resize(4,3);
  F_bar.setZero();

  F_bar.block<1,3>(0,0) = C*A;
  F_bar.block<3,3>(1,0) = A;
  
  I_bar.setZero();
  I_bar(0,0) = 1.0;

  A_bar.block<4,1>(0,0) = I_bar;
  A_bar.block<4,3>(0,1) = F_bar;
  
  Eigen::MatrixXd Qe;
  Qe.resize(1,1);
  Qe(0,0) = 1.0;

  Eigen::MatrixXd R;
  R.resize(1,1);
  R(0,0) = 0.000001;

  Eigen::MatrixXd Qx;
  Qx.resize(3,3);
  Qx.setZero();

  Eigen::MatrixXd Q_bar;
  Q_bar.resize(3,3);
  Q_bar.setZero();
  Q_bar(0,0) = Qe(0,0);

  Eigen::Matrix4d K;
  
  K(0,0) = 110.946733178638; 
  K(0,1) = 6099.115434920124;  
  K(0,2) = 1670.206808355727; 
  K(0,3) = 4.277193660725; 
  K(1,0) = K(0,1); 
  K(1,1) = 342635.571113037353; 
  K(1,2) = 93854.611649038387; 
  K(1,3) = 247.309059159967; 
  K(2,0) = K(0,2); 
  K(2,1) = K(1,2); 
  K(2,2) = 25708.928259919834; 
  K(2,3) = 67.827431998836; 
  K(3,0) = K(0,3); 
  K(3,1) = K(1,3); 
  K(3,2) = K(2,3); 
  K(3,3) = 0.202193033700;

  Eigen::MatrixXd Temp_mat;
  Eigen::MatrixXd Temp_mat_inv;
  Eigen::MatrixXd Ac_bar;
  Temp_mat.resize(1,1);
  Temp_mat.setZero();
  Temp_mat_inv.resize(1,1);
  Temp_mat_inv.setZero();
  Ac_bar.setZero();
  Ac_bar.resize(4,4);

  Temp_mat = R + B_bar_tran * K * B_bar;
  Temp_mat_inv = Temp_mat.inverse();
  
  Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;
  
  Eigen::MatrixXd Ac_bar_tran(4,4);
  Ac_bar_tran = Ac_bar.transpose();
  
  Gi.resize(1,1); Gx.resize(1,3);
  Gi = Temp_mat_inv * B_bar_tran * K * I_bar ;
  Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;   
  
  Eigen::MatrixXd X_bar;
  Eigen::Vector4d X_bar_col;
  X_bar.resize(4, NL); 
  X_bar.setZero();
  X_bar_col.setZero();
  X_bar_col = - Ac_bar_tran * K * I_bar;

  for(int i = 0; i < NL; i++)
  {
    X_bar.block<4,1>(0,i) = X_bar_col;
    X_bar_col = Ac_bar_tran*X_bar_col;
  }           

  Gd.resize(NL);
  Eigen::VectorXd Gd_col(1);
  Gd_col(0) = -Gi(0,0);
  
  for(int i = 0; i < NL; i++)
  {
    Gd.segment(i,1) = Gd_col;
    Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i) ;
  }
}

void WalkingController::previewcontroller(double dt, int NL, int tick, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::MatrixXd C, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{
  int zmp_size;
  zmp_size = ref_zmp_.col(1).size(); // 보행 중 720개 (240 * 3)
  Eigen::VectorXd px_ref, py_ref;
  px_ref.resize(zmp_size);
  py_ref.resize(zmp_size);
  
  for(int i = 0; i < zmp_size; i++)
  {
    px_ref(i) = 0; 
    if((int)current_step_num_ % 2 == 0)
    {
      py_ref(i) = -0.129425000000000; 
    }
    else if((int)current_step_num_ % 2 == 1)
    {
      py_ref(i) = 0.129425000000000;
    }
  }
     
  Eigen::VectorXd px, py;
  px.resize(1); py.resize(1);
  
  if(tick == 0 && current_step_num_ == 0)
  { 
    preview_x_b(0) = x_i;  
    preview_y_b(0) = y_i;
    preview_x(0) = x_i;
    preview_y(0) = y_i;
  }
  else
  {     
    preview_x = xs; preview_y = ys;   
    preview_x_b(0) = preview_x(0) - preview_x(1)*0.005;  
    preview_y_b(0) = preview_y(0) - preview_y(1)*0.005;
    preview_x_b(1) = preview_x(1) - preview_x(2)*0.005;
    preview_y_b(1) = preview_y(1) - preview_y(2)*0.005;
    preview_x_b(2) = preview_x(2) - UX*0.005;
    preview_y_b(2) = preview_y(2) - UY*0.005; 
    
    
  }      
  px = C*preview_x;
  py = C*preview_y; 
  double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

  for(int i = 0; i < NL; i++)
  {
    sum_Gd_px_ref = sum_Gd_px_ref + Gd(i)*(px_ref(tick + 1 + i) - px_ref(tick + i));
    sum_Gd_py_ref = sum_Gd_py_ref + Gd(i)*(py_ref(tick + 1 + i) - py_ref(tick + i));
  }
 
  Eigen::MatrixXd del_ux(1,1);
  Eigen::MatrixXd del_uy(1,1);
  del_ux.setZero();
  del_uy.setZero();
   
  Eigen::VectorXd GX_X(1); 
  GX_X = Gx * (preview_x - preview_x_b);
  Eigen::VectorXd GX_Y(1); 
  GX_Y = Gx * (preview_y - preview_y_b); 
  
  del_ux(0,0) = -(p_d(0))*Gi(0,0) - GX_X(0) - sum_Gd_px_ref;
  del_uy(0,0) = -(p_d(1))*Gi(0,0) - GX_Y(0) - sum_Gd_py_ref;
   
  UX = UX + del_ux(0,0);
  UY = UY + del_uy(0,0);

  XD = A*preview_x + B*UX;
  YD = A*preview_y + B*UY;  
  
}

void WalkingController::getCPTrajectory_M()
{  
  if(walking_tick_ == 0)
  {
    dT(0) = t_total_/hz_;
    dT(1) = t_total_/hz_;
    cp(0) = com_support_init_(0); 
    cp(1) = com_support_init_(1);
    com(0) = com_support_init_(0);
    com(1) = com_support_init_(1);
    com_pre(0) = com_support_init_(0);
    com_pre(1) = com_support_init_(1);
   
    cp_eos(0) = 0.0;
    cp_eos(1) = com_support_init_(1);

    // CP control
    K1(0) = 1.0;
    K1(1) = 0.15;
    K2(0) = 1.0;
    K2(1) = 0.6;
    dT_cp = 50;

    zmp_intergral_control.setZero();
  }
  // CP_eos 설정 /////////////////////////////////////////////////////////////////////
  // Step change 하면, 현재 발 기준으로 다음스텝에 CP_eos를 찍음. (보행 중)
  if( walking_tick_ >= t_start_ && walking_tick_ < t_start_ + t_rest_init_ + t_double1_ ) // First DSP까지는 CP eos를 바꾸면 안됨.
  {
    cp_eos(0) = foot_step_support_frame_(current_step_num_-1,0) ;    
    cp_eos(1) = foot_step_support_frame_(current_step_num_-1,1) ;
  }
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ ) // SSP부터 CP eos를 바꾸면 됨.
  {    
    cp_eos(0) = foot_step_support_frame_(current_step_num_,0) ;
    cp_eos(1) = foot_step_support_frame_(current_step_num_,1) ; 
  }
  // 첫번째 step의 eos 설정.
  if(current_step_num_ == 0)
  {
    if(walking_tick_ >= t_start_ && walking_tick_ < t_start_ +  t_double1_ + t_rest_init_) // First DSP까지 X방향은 0
    { cp_eos(0) = 0; } 
    else if(walking_tick_ >= t_start_ +  t_double1_ + t_rest_init_ && walking_tick_ < t_start_ + t_total_) // SSP 부터 X방향은 다음 발로 
    { cp_eos(0) = 0.2; } 

    if(walking_tick_ >= t_start_ - 0.5*hz_ && walking_tick_ < t_start_ +  t_double1_ + t_rest_init_) // 0.5초전부터 CoM을 미리 움직이기위함
    { cp_eos(1) = 0.0; }
    else if(walking_tick_ >= t_start_ +  t_double1_ + t_rest_init_ && walking_tick_ < t_start_ + t_total_)
    { cp_eos(1) = - 0.25562; } 
  }
  // 마지막 step의 eos 설정.
  if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_start_ && walking_tick_ < t_start_ + t_rest_init_ + t_double1_)
  {
    cp_eos(0) = 0;
    cp_eos(1) = 0;
  }
  else if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_start_ + t_rest_init_ + t_double1_)
  {
    cp_eos(0) = 0;
    cp_eos(1) = 0.12779; 
  }

  // dT 설정 
  if(walking_tick_ == t_start_ + t_rest_init_ + t_double1_)
  {
    dT(0) = t_total_/hz_;
    dT(1) = t_total_/hz_;
  }
  else if(current_step_num_ == 0 && walking_tick_ == 0)
  { dT(0) = 1.0; }
  else if(current_step_num_ == 0 && walking_tick_ == t_start_ - 0.5*hz_)
  { dT(1) = 0.5 + t_rest_init_/hz_; }

  // 좌표 변환
  if(walking_tick_ == t_start_ && current_step_num_ > 0 && current_step_num_ != total_step_num_ -1)  
  {
    Eigen::Vector2d temp_pos;
    for(int i=0; i<3; i++)
    {
      temp_pos(i) = foot_step_support_frame_(current_step_num_,i);
    }
    cp(0) = cp(0) - temp_pos(0);
    cp(1) = cp(1) + temp_pos(1); 
    com_pre(0) = com_pre(0) - temp_pos(0);
    com_pre(1) = com_pre(1) + temp_pos(1);   
  }
  else if(walking_tick_ == t_start_ && current_step_num_ == total_step_num_-1)
  {
    cp(0) = cp(0) - 0.2;
    cp(1) = cp(1) + 0.127794*2;
    com_pre(0) = com_pre(0) - 0.2;
    com_pre(1) = com_pre(1) + 0.127794*2;
  }
  
  // CP Control ////////////////////////////////////////////////////////////////////////////
  b(0)= exp(3.7*dT(0)); 
  p(0) = 1/(1-b(0)) * cp_eos(0) - b(0)/(1-b(0)) * cp(0); // X ZMP calculation

  cp_pre(0) = cp(0); 
  cp(0) = p(0) + exp(3.7*0.005)*(cp_pre(0)-p(0)); // X CP update
  com(0) = 1/(1+3.7*0.005) * com_pre(0) + cp(0) * (3.7*0.005)/(1+3.7*0.005); // X CoM update
  com_pre(0) = com(0);

  // CP HW9
  cp_measured(0) = com_support_current_(0) + com_support_current_dot(0)/3.7;
  b_cp(0)= exp(3.7*dT_cp); 
  p_cp(0) = 1/(1-b_cp(0)) * cp(0) - b_cp(0)/(1-b_cp(0)) * cp_measured(0); // X ZMP calculation for CP


  dT(0) = dT(0) - del_t; // X dT update
  if(dT(0) < 0.01)
  { dT(0) = 0.01; } // X dT Limit
 
  if(walking_tick_ >= 500)
  {
    b(1)= exp(3.7*dT(1));
    p(1) = 1/(1-b(1)) * cp_eos(1) - b(1)/(1-b(1)) * cp(1); // Y ZMP calculation

    cp_pre(1) = cp(1);  
    cp(1) = p(1) + exp(3.7*0.005)*(cp_pre(1)-p(1)); // Y CP update
    com(1) = 1/(1+3.7*0.005) * com_pre(1) + cp(1) * (3.7*0.005)/(1+3.7*0.005); // Y CoM update
    com_pre(1) = com(1);

    // CP HW9
    cp_measured(1) = com_support_current_(1) + com_support_current_dot(1)/3.7;
    b_cp(1)= exp(3.7*dT_cp);
    p_cp(1) = 1/(1-b_cp(1)) * cp(1) - b_cp(1)/(1-b_cp(1)) * cp_measured(1); // Y ZMP calculation for CP


    dT(1) = dT(1) - del_t;
    if(dT(1) < 0.01)
    { dT(1) = 0.01; } // Y dT Limit
  } else {
    cp_measured(1) = com_support_current_(1) + com_support_current_dot(1)/3.7;
  }
 

  // CP Control - HW9 ///////////////////////////////////////////////////////////////////////
  // zmp_intergral_control += p_cp - zmp_measured_;
  // pelv_support_current_.translation()(0) = pelv_support_current_.translation()(0) + K1(0) * (com(0) - com_support_current_(0)) - K2(0) * zmp_intergral_control(0) * dT_cp;
  // pelv_support_current_.translation()(0) = pelv_support_current_.translation()(1) + K1(1) * (com(1) - com_support_current_(1)) - K2(1) * zmp_intergral_control(1) * dT_cp;

    
  MJ_graph << p_cp(0) << "," << cp(0) << "," << cp_measured(0) << "," << zmp_measured_(0) << "," << p_cp(1) << "," << cp(1) << "," << cp_measured(1) << "," << zmp_measured_(1) << endl;
 
  
}

void WalkingController::getComTrajectory()
{
  if(walking_tick_ == 0)  
  { preview_Parameter(1.0/hz_, 16*hz_/10, Gi_, Gd_, Gx_, A_, B_, C_); }

  if(current_step_num_ == 0)
  { zmp_start_time_ = 0.0; }
  else
  { zmp_start_time_ = t_start_; }
          
  previewcontroller(1.0/hz_, 16*hz_/10, walking_tick_-zmp_start_time_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, C_, xd_, yd_);
  
  xs_ = xd_; ys_ = yd_;

  com_desired_(0) = com(0);
  com_desired_(1) = com(1);
  com_desired_(2) = pelv_support_start_.translation()(2);
  
}

void WalkingController::getPelvTrajectory()
{
  double z_rot = foot_step_support_frame_(current_step_num_,5);
   
  pelv_trajectory_support_.translation()(0) = pelv_support_current_.translation()(0) + 1.0*(com_desired_(0) - com_support_current_(0)) ;
  pelv_trajectory_support_.translation()(1) = pelv_support_current_.translation()(1) + 1.0*(com_desired_(1) - com_support_current_(1)) ;
  pelv_trajectory_support_.translation()(2) = com_desired_(2);          
       
  Eigen::Vector3d Trunk_trajectory_euler;
  Trunk_trajectory_euler.setZero();

  if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_)
  { Trunk_trajectory_euler(2) = pelv_support_euler_init_(2); }
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_ < t_start_ + t_total_ - t_double2_ - t_rest_last_)
  { Trunk_trajectory_euler(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_double2_-t_rest_last_, pelv_support_euler_init_(2),z_rot/2.0,0.0,0.0); }
  else
  { Trunk_trajectory_euler(2) = z_rot/2.0; } 
  
  pelv_trajectory_support_.linear() = DyrosMath::rotateWithZ(Trunk_trajectory_euler(2))*DyrosMath::rotateWithY(Trunk_trajectory_euler(1))*DyrosMath::rotateWithX(Trunk_trajectory_euler(0));
}

void WalkingController::supportToFloatPattern()
{  
  pelv_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*pelv_trajectory_support_;
  lfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*lfoot_trajectory_support_;
  rfoot_trajectory_float_ = DyrosMath::inverseIsometry3d(pelv_trajectory_support_)*rfoot_trajectory_support_;
}

void WalkingController::getFootTrajectory()
{
  Eigen::Vector6d target_swing_foot;

  for(int i=0; i<6; i++)
  { target_swing_foot(i) = foot_step_support_frame_(current_step_num_,i); }
 
  if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_) 
  { 
    lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();  
    rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
    
    lfoot_trajectory_euler_support_ = lfoot_support_euler_init_;
    rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;

    if(foot_step_(current_step_num_,6) == 1)  
    { 
      lfoot_trajectory_support_.translation().setZero();
      lfoot_trajectory_euler_support_.setZero();

      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation();
      rfoot_trajectory_support_.translation()(2) = 0;
      rfoot_trajectory_euler_support_ = rfoot_support_euler_init_;
    }     
    else if(foot_step_(current_step_num_,6) == 0)  
    { 
      rfoot_trajectory_support_.translation().setZero();
      rfoot_trajectory_euler_support_.setZero();

      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();
      lfoot_trajectory_support_.translation()(2) = 0;
      lfoot_trajectory_euler_support_ = lfoot_support_euler_init_; 
    }     

    lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
  }
  
  else if(walking_tick_ >= t_start_ + t_rest_init_ + t_double1_ && walking_tick_ < t_start_ + t_total_ - t_double2_ - t_rest_last_)  
  {   
    if(foot_step_(current_step_num_,6) == 1) 
    {
      lfoot_trajectory_support_.translation() = lfoot_support_init_.translation();             
      lfoot_trajectory_euler_support_.setZero(); 

      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0) 
      { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_+ t_rest_init_ + t_double1_,t_start_real_ + t_double1_ + (t_total_ - t_rest_init_ - t_rest_last_ - t_double1_ - t_double2_)/2.0,0,foot_height_,0.0,0.0); }  
      else
      { rfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
      
      for(int i=0; i<2; i++)  
      { rfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_ + t_double1_ , t_start_+t_total_-t_rest_last_-t_double2_, rfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); } 
      
      rfoot_trajectory_euler_support_(0) = 0;
      rfoot_trajectory_euler_support_(1) = 0;
      rfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_ + t_rest_init_ + t_double1_,t_start_ + t_total_ - t_rest_last_ - t_double2_,rfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if(foot_step_(current_step_num_,6) == 0)  
    { 
      rfoot_trajectory_support_.translation() = rfoot_support_init_.translation(); 
      rfoot_trajectory_euler_support_.setZero(); 

      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
 
      if(walking_tick_ < t_start_ + t_rest_init_ + t_double1_ + (t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0)
      { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,0,foot_height_,0.0,0.0); }
      else
      { lfoot_trajectory_support_.translation()(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_+(t_total_-t_rest_init_-t_rest_last_-t_double1_-t_double2_)/2.0,t_start_+t_total_-t_rest_last_-t_double2_,foot_height_,target_swing_foot(2),0.0,0.0); }
         
      for(int i=0; i<2; i++)
      { lfoot_trajectory_support_.translation()(i) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_init_.translation()(i),target_swing_foot(i),0.0,0.0); }

      lfoot_trajectory_euler_support_(0) = 0;
      lfoot_trajectory_euler_support_(1) = 0;  
      lfoot_trajectory_euler_support_(2) = DyrosMath::cubic(walking_tick_,t_start_real_+t_double1_,t_start_+t_total_-t_rest_last_-t_double2_,lfoot_support_euler_init_(2),target_swing_foot(5),0.0,0.0);
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    } 
  }
  else 
  { 
    if(foot_step_(current_step_num_,6) == 1) 
    { 
      lfoot_trajectory_euler_support_.setZero();
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
      
      for(int i=0; i<3; i++)
      {
        rfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        rfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));
    }
    else if (foot_step_(current_step_num_,6) == 0) 
    { 
      rfoot_trajectory_euler_support_.setZero();
      rfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(rfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(rfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(rfoot_trajectory_euler_support_(0));

      for(int i=0; i<3; i++)
      {
        lfoot_trajectory_support_.translation()(i) = target_swing_foot(i);
        lfoot_trajectory_euler_support_(i) = target_swing_foot(i+3);
      }
      lfoot_trajectory_support_.linear() = DyrosMath::rotateWithZ(lfoot_trajectory_euler_support_(2))*DyrosMath::rotateWithY(lfoot_trajectory_euler_support_(1))*DyrosMath::rotateWithX(lfoot_trajectory_euler_support_(0));
    }
  }
}

void WalkingController::computeIkControl_MJ(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des)
{
   
  double offset_hip_pitch = 24.0799945102*DEG2RAD;
  double offset_knee_pitch = 14.8197729791*DEG2RAD;
  double offset_ankle_pitch = 9.2602215311*DEG2RAD;

  Eigen::Vector3d R_r, R_D, L_r, L_D ;

  L_D << 0 , +0.105, -0.1119;
  R_D << 0 , -0.105, -0.1119;
  
  L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*L_D - float_lleg_transform.translation());
  R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation()*R_D - float_rleg_transform.translation());
  
  double R_C = 0, L_C = 0, L_upper = 0.3713, L_lower = 0.3728 , R_alpha = 0, L_alpha = 0;

  L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
  R_C = sqrt( pow(R_r(0),2) + pow(R_r(1),2) + pow(R_r(2),2) );
  
  q_des(3) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(L_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
  q_des(9) = (-acos((pow(L_upper,2) + pow(L_lower,2) - pow(R_C,2)) / (2*L_upper*L_lower))+ M_PI) ;
  L_alpha = asin(L_upper / L_C * sin(M_PI - q_des(3)));
  R_alpha = asin(L_upper / R_C * sin(M_PI - q_des(9)));

  q_des(4)  = -atan2(L_r(0), sqrt(pow(L_r(1),2) + pow(L_r(2),2))) - L_alpha ;
  q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha ;
   
  Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
  Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
  Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

  L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3)-q_des(4));
  L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
  R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9)-q_des(10));
  R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11)); 
  
  L_Hip_rot_mat.setZero(); R_Hip_rot_mat.setZero();

  L_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_lleg_transform.rotation() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat; 
  R_Hip_rot_mat = float_trunk_transform.rotation().transpose() * float_rleg_transform.rotation() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

  q_des(0) = -atan2(-L_Hip_rot_mat(0,1),L_Hip_rot_mat(1,1)); // Hip yaw
  q_des(1) =  atan2(L_Hip_rot_mat(2,1), -L_Hip_rot_mat(0,1) * sin(q_des(0)) + L_Hip_rot_mat(1,1)*cos(q_des(0))); // Hip roll
  q_des(2) =  atan2(-L_Hip_rot_mat(2,0), L_Hip_rot_mat(2,2)) + offset_hip_pitch; // Hip pitch
  q_des(3) =  q_des(3) - 14.8197729791*DEG2RAD; // Knee pitch
  q_des(4) =  q_des(4) - 9.2602215311*DEG2RAD; // Ankle pitch
  q_des(5) =  atan2( L_r(1), L_r(2) ); // Ankle roll

  q_des(6) = -atan2(-R_Hip_rot_mat(0,1),R_Hip_rot_mat(1,1));
  q_des(7) =  atan2(R_Hip_rot_mat(2,1), -R_Hip_rot_mat(0,1) * sin(q_des(6)) + R_Hip_rot_mat(1,1)*cos(q_des(6)));
  q_des(8) = -atan2(-R_Hip_rot_mat(2,0), R_Hip_rot_mat(2,2)) - offset_hip_pitch;
  q_des(9) = -q_des(9) + 14.8197729791*DEG2RAD;
  q_des(10) = -q_des(10) + 9.2602215311*DEG2RAD; 
  q_des(11) =  atan2( R_r(1), R_r(2) );
  
}

void WalkingController::updateNextStepTime()
{
  if(walking_tick_ == t_last_) 
  { 
    if(current_step_num_ != total_step_num_-1)
    { 
      t_start_ = t_last_ + 1 ; 
      t_start_real_ = t_start_ + t_rest_init_; 
      t_last_ = t_start_ + t_total_ -1;   
      current_step_num_ ++; 
    }    
  }
   if(current_step_num_ == total_step_num_-1 && walking_tick_ >= t_last_ + t_total_)
   {
     walking_enable_ = false;
   }
   walking_tick_ ++;
}

void WalkingController::slowCalc()
{
  while(true)
  {
    if(ready_for_thread_flag_)
    {
       
      if (ready_for_compute_flag_ == false)
      {
        ready_for_compute_flag_ = true;
      }
    }
    this_thread::sleep_for(chrono::milliseconds(100));
  }
} 

void WalkingController::hip_compensator()
{
    double left_hip_angle = 1.0*DEG2RAD, right_hip_angle = 1.0*DEG2RAD, left_hip_angle_first_step = 1.0*DEG2RAD, right_hip_angle_first_step = 1.0*DEG2RAD,
      left_hip_angle_temp = 0.0, right_hip_angle_temp = 0.0, temp_time = 0.1*hz_;

  if(current_step_num_ == 0)
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        left_hip_angle_temp = 0;
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_angle_first_step, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_angle_first_step, 0.0, 0.0, 0.0);
      else
        right_hip_angle_temp = 0;
    }
  }
  else
  {
    if(foot_step_(current_step_num_, 6) == 1) //left support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, left_hip_angle, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        left_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,left_hip_angle, 0.0, 0.0, 0.0);
      else
        left_hip_angle_temp = 0;
    }
    else if(foot_step_(current_step_num_, 6) == 0) // right support foot
    {
      if(walking_tick_ < t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_rest_init_ + t_double1_, t_start_ + t_rest_init_ + t_double1_ + temp_time, 0, right_hip_angle, 0.0, 0.0);
      else if(walking_tick_ >= t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time)
        right_hip_angle_temp = DyrosMath::cubic(walking_tick_, t_start_ + t_total_ - t_rest_last_ - t_double2_ - temp_time, t_start_ + t_total_ - t_rest_last_,right_hip_angle, 0.0, 0.0, 0.0);
      else
        right_hip_angle_temp = 0;
    }
  }
  desired_q_(1) = desired_q_(1) + left_hip_angle_temp;
  desired_q_(7) = desired_q_(7) - right_hip_angle_temp;
}

}
 
