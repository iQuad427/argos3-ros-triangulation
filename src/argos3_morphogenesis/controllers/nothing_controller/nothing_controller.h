/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
// Include random number generator
#include <argos3/core/utility/math/rng.h>

#include <iostream>
#include <vector>
#include <utility>
#include <cmath>

/* ROS dependencies */
#include "ros/ros.h"
#include <morpho_msgs/Direction.h>
#include <morpho_msgs/Angle.h>
#include <tri_msgs/Distance.h>
#include <tri_msgs/Distances.h>

#define STOP 0
#define MOVE 1
#define TURN 2
#define TURN_L 21
#define TURN_R 22
#define GO   3
#define AVOID 4
#define PI 3.14159265

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotBase : public CCI_Controller {

public:

    /* Class constructor. */
    CFootBotBase();

    /* Class destructor. */
    ~CFootBotBase() override {}

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><footbot_diffusion_controller> section.
     */
    void Init(TConfigurationNode& t_node) override;

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    void ControlStep() override;

    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     * In this example controller there is no need for resetting anything,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    void Reset() override;

    /*
     * Called to clean up what is done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    void Destroy() override {}

private:

};

#endif
