#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "Simbody.h"
#include <OpenSim/OpenSim.h>
#include <stdio.h>
#include <time.h>
#include "body_tracker_msgs/Joints.h"

using namespace SimTK;
using namespace OpenSim;
using namespace std;

ros::Publisher model_pubs;
OpenSim::Model model("/home/nakano/Projects/utcn/beterrehab/osim_models/DAS3_release2/OpenSim_model/das3.osim");
const float rate = 100.0; //In Hz
float sim_time = 0.0;
Set<Muscle>& muscles  = model.updMuscles();
PrescribedController* brain = new PrescribedController();

std::string get_date(void)
{
  time_t now;
  char the_date[16];

  the_date[0] = '\0';

  now = time(NULL);

  if (now != -1)
    {
      strftime(the_date, 16, "%F_%H%M", gmtime(&now));
    }

  return std::string(the_date);
}

string timestamp = get_date();

void kineticsCallback(const std_msgs::Float32::ConstPtr& msg, OpenSim::Manager* manager)
{
  float biceps_control = abs(0.7*sin(1.8/3.141592653*msg->data));
  float triceps_control = abs(0.9*sin(1.8/3.14*msg->data+3.14/2));
  State& state = model.updWorkingState();

  brain->prescribeControlForActuator("bic_l", new Constant(biceps_control));
  brain->prescribeControlForActuator("bic_b_1", new Constant(biceps_control));
  brain->prescribeControlForActuator("bic_b_2", new Constant(biceps_control));
  brain->prescribeControlForActuator("tric_long_1", new Constant(triceps_control));
  brain->prescribeControlForActuator("tric_long_2", new Constant(triceps_control));
  brain->prescribeControlForActuator("tric_long_3", new Constant(triceps_control));
  brain->prescribeControlForActuator("tric_long_4", new Constant(triceps_control));

  // Simulate and update visualization
  sim_time = sim_time + 1/rate;
  manager->integrate(sim_time);

  // auto controlsTable = model.getControlsTable();
  // STOFileAdapter::write(controlsTable, "/home/nakano/Projects/utcn/beterrehab/model_output/controls_"+timestamp+".sto");

  // auto statesTable = manager->getStatesTable();
  // model.updSimbodyEngine().convertRadiansToDegrees(statesTable);
  // STOFileAdapter::write(statesTable, "/home/nakano/Projects/utcn/beterrehab/model_output/states_degrees_"+timestamp+".sto");
}

void kinematicsCallback(const body_tracker_msgs::Joints::ConstPtr& msg, OpenSim::Manager* manager)
{
  State& state = model.updWorkingState();
  Joint& hum = model.updJointSet().get("hu");
  Joint& gh1 = model.updJointSet().get("gh1");
  Joint& gh2 = model.updJointSet().get("gh2");
  Joint& gh3 = model.updJointSet().get("gh3");

  hum.updCoordinate().setValue(state, msg->elbow_r_flex);
  gh1.updCoordinate().setValue(state, msg->shoulder_r_abd);
  gh2.updCoordinate().setValue(state, msg->shoulder_r_rot);
  gh3.updCoordinate().setValue(state, msg->shoulder_r_flex);

  model.updVisualizer().show(state);

  ROS_INFO("Updated elbow flexion to %f", msg->elbow_r_flex);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  model.setUseVisualizer(true);
  model.setGravity(Vec3(0,-1,0));

  brain->addActuator(muscles.get("bic_l"));
  brain->addActuator(muscles.get("bic_b_1"));
  brain->addActuator(muscles.get("bic_b_2"));
  brain->addActuator(muscles.get("tric_long_1"));
  brain->addActuator(muscles.get("tric_long_2"));
  brain->addActuator(muscles.get("tric_long_3"));
  brain->addActuator(muscles.get("tric_long_4"));

  brain->prescribeControlForActuator("bic_l", new Constant(0.8));
  brain->prescribeControlForActuator("bic_b_1", new Constant(0.8));
  brain->prescribeControlForActuator("bic_b_2", new Constant(0.8));
  brain->prescribeControlForActuator("tric_long_1", new Constant(0.2));
  brain->prescribeControlForActuator("tric_long_2", new Constant(0.2));
  brain->prescribeControlForActuator("tric_long_3", new Constant(0.2));
  brain->prescribeControlForActuator("tric_long_4", new Constant(0.2));

  model.addController(brain);

  ROS_INFO("Model controller is ready");

  Joint& hum = model.updJointSet().get("hu");
  Joint& gh1 = model.updJointSet().get("gh1");
  Joint& gh2 = model.updJointSet().get("gh2");
  Joint& gh3 = model.updJointSet().get("gh3");

  ForceReporter* forces = new ForceReporter(&model);
  model.updAnalysisSet().adoptAndAppend(forces);

  ROS_INFO("Reporter is ready");

  State& state = model.initSystem();

  // Fix all the joints besides the humerus joint.
  for (int i=0; i < model.getNumJoints(); i++) {
    Joint& joint = model.updJointSet().get(i);
    if (joint.numCoordinates() > 0) {
      ROS_INFO("I am locking joint %s", joint.getName().c_str() );
      joint.updCoordinate().setLocked(state,true);
    }
  }
  hum.updCoordinate().setLocked(state,false);
  gh1.updCoordinate().setLocked(state,false);
  gh2.updCoordinate().setLocked(state,false);
  gh3.updCoordinate().setLocked(state,false);

  model.equilibrateMuscles(state);

  ROS_INFO("All joints initialized");

  Manager manager(model,state);
  manager.setIntegratorAccuracy(1.0e-4);

  ROS_INFO("Manager is ready");

  // Configure the visualizer.
  model.updMatterSubsystem().setShowDefaultGeometry(true);
  Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
  viz.setBackgroundType(viz.SolidColor);
  viz.setBackgroundColor(White);
  viz.setShowSimTime(true);

  model_pubs = n.advertise<std_msgs::Float32>("state", 1000/rate);
  ros::Subscriber sub_orb = n.subscribe<body_tracker_msgs::Joints>("/body_tracker/joints", 1000/rate, boost::bind(kinematicsCallback, _1, &manager));
  ros::Subscriber sub_emg = n.subscribe<std_msgs::Float32>("/mass", 1000/rate, boost::bind(kineticsCallback, _1, &manager));

  ros::MultiThreadedSpinner spinner(4);
  ros::spin();

  return 0;
}
