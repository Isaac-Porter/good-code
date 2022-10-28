using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern motor Shooter;
extern motor Intake;
extern motor test;
extern digital_out loader;
extern encoder lateralEnc;
extern encoder straightEnc;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );