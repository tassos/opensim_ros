// Libraries
#include "ros/ros.h"
#include "Simbody.h"
#include "OpenSim.h"

// Messages
#include "osim_msgs/fkine.h"

using namespace SimTK;
using namespace OpenSim;
using namespace std;

ros::Publisher model_pubs;
OpenSim::Model model("location_of_the_OpenSim_model/das3.osim");

// Variables
const float rate = 1.0; //In Hz
double sim_time = 0.0;
Set<Muscle>& muscles  = model.updMuscles();
PrescribedController* brain = new PrescribedController();

void fkineCallback(const osim_msgs::fkine::ConstPtr& msg, OpenSim::StatesTrajectoryReporter* reporter)
{
  State& state = model.initializeState();

  // Fix all the joints
  for (int i=0; i < model.getNumJoints(); i++) {
    Joint& joint = model.updJointSet().get(i);
    if (joint.numCoordinates() > 0) {
      ROS_INFO("I am locking joint %s", joint.getName().c_str() );
      joint.updCoordinate().setLocked(state,true);
    }
  }

  model.equilibrateMuscles(state);
  Joint& hum = model.updJointSet().get("hu");
  Joint& gh1 = model.updJointSet().get("gh1");
  Joint& gh2 = model.updJointSet().get("gh2");
  Joint& gh3 = model.updJointSet().get("gh3");

  // Release the arm joints
  hum.updCoordinate().setLocked(state,false);
  // gh1.updCoordinate().setLocked(state,false);
  // gh2.updCoordinate().setLocked(state,false);
  // gh3.updCoordinate().setLocked(state,false);

  // Extract initial conditions
  vector<double> ef = msg->elbow_flex.data;
  double *elbow_flex = &ef[0];
  vector<double> sabd = msg->shoulder_abd.data;
  double *shoulder_abd = &sabd[0];
  vector<double> sr = msg->shoulder_rot.data;
  double *shoulder_rot = &sr[0];
  vector<double> sf = msg->shoulder_flex.data;
  double *shoulder_flex = &sf[0];

  // Enforce initial conditions for position
  hum.updCoordinate().setValue(state, elbow_flex[0]);
  gh1.updCoordinate().setValue(state, shoulder_abd[0]);
  gh2.updCoordinate().setValue(state, shoulder_rot[0]);
  gh3.updCoordinate().setValue(state, shoulder_flex[0]);

  // Enforce initial conditions for speed
  hum.updCoordinate().setSpeedValue(state, msg->vel_elbow_flex.data);
  gh1.updCoordinate().setSpeedValue(state, msg->vel_shoulder_abd.data);
  gh2.updCoordinate().setSpeedValue(state, msg->vel_shoulder_rot.data);
  gh3.updCoordinate().setSpeedValue(state, msg->vel_shoulder_flex.data);

  // Extract EMG trajectories
  double emg_rate = msg->emg_rate.data;
  vector<double> b = msg->biceps.data;
  double* biceps = &b[0];
  int steps = b.size();

  vector<double> t = msg->triceps.data;
  double* triceps = &t[0];

  // Construct time series
  vector<double> seq;
  for (double i=0; i<steps; i++)
    {
      seq.push_back(i/emg_rate);
    }
  double* times = &seq[0];

  // Apply EMG trajectories
  brain->prescribeControlForActuator("bic_l", new PiecewiseLinearFunction(steps, times, biceps, "Biceps"));
  brain->prescribeControlForActuator("bic_b_1", new PiecewiseLinearFunction(steps, times, biceps, "Biceps"));
  brain->prescribeControlForActuator("bic_b_2", new PiecewiseLinearFunction(steps, times, biceps, "Biceps"));
  brain->prescribeControlForActuator("tric_long_1", new PiecewiseLinearFunction(steps, times, triceps, "Triceps"));
  brain->prescribeControlForActuator("tric_long_2", new PiecewiseLinearFunction(steps, times, triceps, "Triceps"));
  brain->prescribeControlForActuator("tric_long_3", new PiecewiseLinearFunction(steps, times, triceps, "Triceps"));
  brain->prescribeControlForActuator("tric_long_4", new PiecewiseLinearFunction(steps, times, triceps, "Triceps"));

  // Run forward kinematics
  Manager manager(model,state);
  manager.setIntegratorAccuracy(1.0e-4);
  manager.integrate(double(steps-1)/emg_rate);

  reporter->clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fkine");

  ros::NodeHandle n;

  model.setUseVisualizer(false);
  model.setGravity(Vec3(0,-1,0));

  Muscle& bic = muscles.get("bic_l");
  Muscle& tri = muscles.get("tric_long_1");

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

  ConsoleReporter* creporter = new ConsoleReporter();
  creporter->set_report_time_interval(0.1);
  creporter->addToReport(hum.getCoordinate().getOutput("value"), "Hum_angle");
  creporter->addToReport(bic.getOutput("fiber_force"), "biceps");
  creporter->addToReport(tri.getOutput("fiber_force"), "triceps");
  model.addComponent(creporter);

  StatesTrajectoryReporter* sreporter = new StatesTrajectoryReporter();
  sreporter->set_report_time_interval(0.1);
  model.addComponent(sreporter);

  ROS_INFO("Reporters are ready");

  model.buildSystem();

  Configure the visualizer
  model.updMatterSubsystem().setShowDefaultGeometry(true);
  Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
  viz.setBackgroundType(viz.SolidColor);
  viz.setBackgroundColor(White);
  viz.setShowSimTime(true);

  ROS_INFO("All joints initialized");

  model_pubs = n.advertise<osim_msgs::pkine>("/osim/pkine", 1000/rate);
  ros::Subscriber fkine = n.subscribe<osim_msgs::fkine>("/osim/fkine", 1000/rate, boost::bind(fkineCallback, _1, sreporter));

  ros::MultiThreadedSpinner spinner(4);
  ros::spin();

  return 0;
}
