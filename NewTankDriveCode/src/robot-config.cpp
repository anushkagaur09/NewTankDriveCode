#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor rightFront = motor(PORT1, ratio18_1, false);
motor leftFront = motor(PORT3, ratio18_1, false);
motor leftBack = motor(PORT7, ratio18_1, false);
motor rightBack = motor(PORT12, ratio18_1, false);
motor roller = motor(PORT8, ratio18_1, false);
digital_in Expansion = digital_in(Brain.ThreeWirePort.A);
digital_in shooterIndexer = digital_in(Brain.ThreeWirePort.B);
motor Intake = motor(PORT9, ratio18_1, false);
motor Intake2 = motor(PORT10, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}