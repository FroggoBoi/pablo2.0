vex::brain Cerebro;
vex::motor RDriveF = vex::motor(vex::PORT15,vex::gearSetting::ratio18_1,true);
vex::motor RDriveB = vex::motor(vex::PORT1,vex::gearSetting::ratio18_1,true);
vex::motor LDriveF = vex::motor(vex::PORT11,vex::gearSetting::ratio18_1,false);
vex::motor LDriveB = vex::motor(vex::PORT12,vex::gearSetting::ratio18_1,false);

vex::motor Claw = vex::motor(vex::PORT9,vex::gearSetting::ratio18_1,false);
vex::motor ClawJoint = vex::motor(vex::PORT8,vex::gearSetting::ratio18_1,false);
vex::motor LTower = vex::motor(vex::PORT4,vex::gearSetting::ratio36_1,true);
vex::motor RTower = vex::motor(vex::PORT5,vex::gearSetting::ratio36_1,true);

vex::controller RC1 = vex::controller();
vex::controller RC2 = vex::controller(vex::controllerType::partner);


///// Sonar sensors /////
vex::sonar SonarLeft = vex::sonar(Cerebro.ThreeWirePort.A); // Output cable goes into this one
vex::sonar SonarRight = vex::sonar(Cerebro.ThreeWirePort.C); // See above

//////////////////////////////////////////////////////////////////////////////
//    __  __       _          ____            _             _ _             //
//   |  \/  | __ _(_)_ __    / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __   //
//   | |\/| |/ _` | | '_ \  | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|  //
vex::accelerometer Accelerometer = vex::accelerometer(Brain.ThreeWirePort.A);//   | |  | | (_| | | | | | | |__| (_) | | | | |_| | | (_) | | |  __/ |     //
//   |_|  |_|\__,_|_|_| |_|  \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
/*
///// D-Pad /////
Left: 
Right: Toggle drive polarity
Up:
Down: lock/unlock wheels

///// Face Buttons /////
A:
B:
X:
Y:

///// Joysticks /////
Axis1: Turning left / right
Axis2: 
Axis3: Drive forward / backwards
Axis4:

///// Shoulder Buttons /////
L1: Pusher up
L2: Pusher down
R1: Claw Open
R2: Claw Close
*/

/////////////////////////////////////////////////////////////////////////////////////////////
//    ____            _                      ____            _             _ _             //
//   |  _ \ __ _ _ __| |_ _ __   ___ _ __   / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __   //
//   | |_) / _` | '__| __| '_ \ / _ \ '__| | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|  //
//   |  __/ (_| | |  | |_| | | |  __/ |    | |__| (_) | | | | |_| | | (_) | | |  __/ |     //
//   |_|   \__,_|_|   \__|_| |_|\___|_|     \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|     //
//                                                                                         //
/////////////////////////////////////////////////////////////////////////////////////////////
/*
///// D-Pad /////
Left: 
Right: 
Up: Flip Claw & Raise Tower & Then Lower It Back Down(for stealing and for simple cap scoring)  
Down: Flip Claw 180º

///// Face Buttons /////
A: Reset claw encoders
B: Reset clawjoint Encoders
X: Reset tower encoders and TowerCount variable
Y: Set tower rotation to 20, allowing us to carry caps around ***

///// Joysticks /////
Axis1:
Axis2: Tower up and down manual
Axis3:
Axis4: Claw joint manual override

///// Shoulder Buttons /////
L1: Open claw
L2: Close claw
R1: Tower increment
R2: Tower decrement 
*/
