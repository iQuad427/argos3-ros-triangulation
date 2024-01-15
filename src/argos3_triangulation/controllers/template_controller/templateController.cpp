#include "templateController.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

CEPuckRos::CEPuckRos() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  velocity(2.5f),   //default max speed
  speedLeft(0.0f),  //default left wheel speed
  speedRight(0.0f), //default right wheel speed
  min_range(0.002), //default min range for proximity sensors
  max_range(0.07)   //default max range for proximity sensors
  {}


void CEPuckRos::Init(TConfigurationNode& t_node) 
{
  //init actuators/sensors
  m_pcWheels      = GetActuator<CCI_EPuckWheelsActuator              >("epuck_wheels"           );
  m_pcProximity   = GetSensor  <CCI_EPuckProximitySensor             >("epuck_proximity"        );

  //init RNG
  m_pcRng = CRandom::CreateRNG("argos");
  m_cRandomRange.SetMax(1.0);

  //get parameters from .argos file
  GetNodeAttributeOrDefault(t_node, "velocity", velocity, velocity);
  GetNodeAttributeOrDefault(t_node, "min_range", min_range, min_range);
  GetNodeAttributeOrDefault(t_node, "max_range", max_range, max_range);

  //init e-puck speed
  speedLeft = velocity;
  speedRight = velocity;

  //init ROS
  initRos();
}

void CEPuckRos::initRos()
{
  //get e-puck ID
  std::stringstream name;
  name.str("");
  name << GetId();

  //init ROS
  if(!ros::isInitialized())
  {
    char** argv = NULL;
    int argc = 0;
    ros::init(argc, argv, name.str());
  }

  //ROS access node
  ros::NodeHandle rosNode;

  std::stringstream ss;
  
  //init 8 proximity sensors messages
  
  for (int i = 0; i < 8; ++i)
  {
    //the e-puck will publish on /${epuck_ID}/proximity
    ss.str("");
    ss << name.str() << "/proximity";
    proximity_pubs[i] = rosNode.advertise<sensor_msgs::Range>(ss.str(), 10); 

    //set proximity sensor number
    ss.str("");
    ss << name.str() << "/base_prox" << i; 
    proximity_msgs[i].header.frame_id = ss.str();

    //set sensor parameters
    proximity_msgs[i].radiation_type = sensor_msgs::Range::INFRARED;
    proximity_msgs[i].field_of_view = 0.26;
    proximity_msgs[i].min_range = min_range;
    proximity_msgs[i].max_range = max_range;
  }

}

void CEPuckRos::ControlStep() 
{
  if(ros::ok())
  {
    
    //do stuff: for example, ask the robot to move 
    m_pcWheels->SetLinearVelocity(speedLeft, speedRight);

    //send ROS messages
    rosControlStep();
    
    //update ROS status
    ros::spinOnce();
  }
}

void CEPuckRos::rosControlStep() 
{
  std::stringstream name;
  name.str("");
  name << GetId();

  //send 8 proximity messages
  for(int i=0; i<8; i++) 
  {
    //0 means there is nothing, 1 means the e-puck is right next the obstacle
    if(m_pcProximity->GetReadings()[i].Value > 0) 
    {
        //translate measurement into real distance (in m)
        proximity_msgs[i].range = (1-sqrt(m_pcProximity->GetReadings()[i].Value))*proximity_msgs[i].max_range;
    } 
    else 
    {
        proximity_msgs[i].range = proximity_msgs[i].max_range;
    }

    //normalize out of bounds measurements
    if(proximity_msgs[i].range > proximity_msgs[i].max_range) 
    {
        proximity_msgs[i].range = proximity_msgs[i].max_range;
    }
    if(proximity_msgs[i].range < proximity_msgs[i].min_range) 
    {
        proximity_msgs[i].range = proximity_msgs[i].min_range;
    }

    //set timestamp and publish
    proximity_msgs[i].header.stamp = ros::Time::now();
    proximity_pubs[i].publish(proximity_msgs[i]);
  }   
}

REGISTER_CONTROLLER(CEPuckRos, "template_controller")
