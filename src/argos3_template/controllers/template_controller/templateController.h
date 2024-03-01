#ifndef TEMPLATECONTROLLERS_H
#define TEMPLATECONTROLLERS_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>

//ros related libraries
#include "ros/ros.h"
#include <sensor_msgs/Range.h>

//epuck dimensions parameters
#define ROBOT_RADIUS 0.035
#define WHEEL_DIAMETER 4       
#define WHEEL_SEPARATION 0.53
#define WHEEL_DISTANCE 0.053  
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0) 
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0) //distance travelled by the e-puck at each simulation step

using namespace argos;


class CEPuckRos : public CCI_Controller {

public:

   CEPuckRos();

   virtual ~CEPuckRos() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

   virtual void initRos(); //must be called in Init

   virtual void rosControlStep(); //must be called in ControlStep

private:
   
   //proximity ros publisher
   ros::Publisher proximity_pubs[8];
   sensor_msgs::Range proximity_msgs[8];

   CCI_EPuckWheelsActuator* m_pcWheels; //wheels
   CCI_EPuckProximitySensor* m_pcProximity; //proximity sensors

   //RNG variables
   CRandom::CRNG* m_pcRng; 
   CRange<Real> m_cRandomRange;
   
   Real speedLeft;
   Real speedRight;
   Real velocity;

   double min_range;
   double max_range;
};

#endif
