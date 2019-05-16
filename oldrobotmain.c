#include "robot-config.h"
#include <vector>

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: 13666A Code                                           */
/*                                                                           */
/*---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
vex::competition    Competition;

/////////////////////////////////////////////////////////////////////////////////////
//     ____ _       _           _  __     __         _       _     _               //
//    / ___| | ___ | |__   __ _| | \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___     //
//   | |  _| |/ _ \| '_ \ / _` | |  \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|    //
//   | |_| | | (_) | |_) | (_| | |   \ V / (_| | |  | | (_| | |_) | |  __/\__ \    //
//    \____|_|\___/|_.__/ \__,_|_|    \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/    //
//                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////

///// Drivng Variables /////
int DrivePolarity = 1; // 1 = claw front; -1 = claw back
bool DriveHold = false; // Boolean to store the current brake status of the drive motors
double StraightDriveMultiplyer; //Multiplyer to control the driving speed when going straight
double TurnDriveMultiplyer; //Multiplyer to control the driving speed when point turning
int MediumDrivePowerThreshold = 55*3; //Value to decide when to run drive train motors at medium power
int LowDrivePowerThreshold = 100*3; //Value to device when to run drive train motors at low power
int Axis1Deadzone = 15; //Deadzone used to control the precision between driving straight and point turning
int Axis3Deadzone = 15;  //Deadzone on controller to make anything happen
int Axis4Deadzone = 40; // Deadzone for strafing
int FullDrivePowerThreshold = 95; //Percent value to make driving straight go at full speed

///// Tower Variables /////
int TowerCount = 0; // Variable used to increment tower height setting
int TowerBottomRotation = 0; // Tower encoder value for all the way down
int LowHangStealRotation = 130*3; // Tower encoder value for stealing caps from the lower posts
int LowHangPlaceRotation = 157*3; // Tower encoder value for placing caps on lower posts
int HighHangStealRotation = 190*3; // Tower encoder value for placing caps on high posts
int TowerMaxRotation = 242*3; // Tower encoder value for full extension (and placing caps on high posts)
bool TowerMovingAutomatically = false; // Tracking if the tower is moving on it's own
int TowerSpeed = 100; // Speed tower should rotate at 
int TowerDirection = 0; // What direction is the tower moving
double TowerSpeedMult = 1; // Multiplyer for moving the tower
int TowerDeadzone = 1; // Deadzone for moving the tower on RC2 Axis 3
bool TowerHold = false;

///// Controller Variables /////
bool APressed = false;
bool XPressed = false;
bool UpPressed = false;
bool LeftPressed = false;
//bool R1Pressed = false;
//bool R2Pressed = false;
bool L1Pressed = false;
bool L2Pressed = false;

///// Claw Variables /////
int ClawSpeed = 100; // Claw opening/closing speed
int ClawOpen = 150; // Encoder value for an open claw
int ClawClosed = 0; //Encoder value for a closed claw
bool ClawMovingAutomatically = false; // 

///// Claw Joint Variables /////
bool ClawJointFlipped = false; //Boolean to tell if the claw is flipped
bool ClawJointMovingAutomatically = false;
int ClawJointSpeed = 60; //Speed claw joint rotates at when moving automatically
double ClawJointManualMultiplyer = 0.1; // Multiplyer for claw joint when moving manually
int ClawJointDeadzone = 10; // Deadzone for manual claw joint movement
//int ClawFlipRotation = 178; //Degree rotation to set claw joint to when flipping
//int ClawResetRotation = 2; //Degree rotation to set claw joint to when resetting

///// Brain Screen Variables /////
int PressButtonStartX;
int PressButtonStartY;
int PressButtonEndX;
int PressButtonEndY;

///// Autonomous Variables /////
bool isBlue; // Variable to store what color the alliance is
bool isClose; // Variable to store what position the robot is set to start in
bool isAny; // Variable to store if the 'any postion' auton is selected

//////////////////////////////////////////////////////////////////////////////////////////
//    ____       _       _               _____                 _   _                    //
//   |  _ \ _ __(_)_   _(_)_ __   __ _  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___    //
//   | | | | '__| \ \ / / | '_ \ / _` | | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|   //
//   | |_| | |  | |\ V /| | | | | (_| | |  _|| |_| | | | | (__| |_| | (_) | | | \__ \   //
//   |____/|_|  |_| \_/ |_|_| |_|\__, | |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/   //
//                               |___/                                                  //
//////////////////////////////////////////////////////////////////////////////////////////

/// Function to toggle the direction of driving ///
void ToggleDrivePolarity()
{
    DrivePolarity = -1 * DrivePolarity; //Switch the polarity
}

/// Function to toggle the brake type of the drive motors ///
void ToggleDriveHold()
{
    DriveHold = !DriveHold;
    if(DriveHold)
    {
        // Set to hold motors //
        LDriveF.setStopping(vex::brakeType::hold);
        LDriveB.setStopping(vex::brakeType::hold);
        RDriveF.setStopping(vex::brakeType::hold);
        RDriveB.setStopping(vex::brakeType::hold);
    }
    else
    {
        // Set to coast motors //
        LDriveF.setStopping(vex::brakeType::coast);
        LDriveB.setStopping(vex::brakeType::coast);
        RDriveF.setStopping(vex::brakeType::coast);
        RDriveB.setStopping(vex::brakeType::coast);
    }
}

///// Main function to control driving /////
int DriveControl() 
{  
    //Axis3 is vertical on left joystick
    //Axis4 is horizontal on left joystick
    //Axis1 is horizontal on right joystick
    //Axis2 is vertical on right joystick
    
    RC1.ButtonRight.pressed(ToggleDrivePolarity); // Setup a callback for the right button on the first controller to toggle drive polarity
    RC1.ButtonDown.pressed(ToggleDriveHold); // Setup a callback for the down button on the first controller to toggle brake mode
    while(1)
    {
        int TowerRotation = RTower.rotation(vex::rotationUnits::deg);
        int RC1Axis1 = RC1.Axis1.position(vex::percentUnits::pct);
        int RC1Axis3 = RC1.Axis3.position(vex::percentUnits::pct);
        int RC1Axis4 = RC1.Axis4.position(vex::percentUnits::pct);
        
        if (TowerRotation > MediumDrivePowerThreshold && TowerRotation < LowDrivePowerThreshold) //If the tower is too high for stable full-speed driving
        {
            //Slow down the drivetrain
            StraightDriveMultiplyer = 0.5;
            TurnDriveMultiplyer = 0.5;
        }
        else if (TowerRotation > LowDrivePowerThreshold) //If the tower is even higher, slow down more
        {
            //Slow down the drivetrain more
            StraightDriveMultiplyer = 0.35;
            TurnDriveMultiplyer = 0.5;
        }
        else //If the tower is low enough for stable full-speed driving
        {
            //Leave the speed muliplyer alone
            StraightDriveMultiplyer = 1; 
            TurnDriveMultiplyer = 1;
        }
        
        /*
        /// Swing turns ///
        if((abs(RC1Axis3) > Axis3Deadzone) && (abs(RC1Axis1) > Axis1Deadzone)) 
        {
            LDrive.spin(vex::directionType::fwd,RC1Axis3,vex::velocityUnits::pct);
            RDrive.spin(vex::directionType::fwd,RC1Axis3,vex::velocityUnits::pct);
        }
        */
        /// Strafing ///
        if(abs(RC1Axis4) > Axis4Deadzone)
        {
            LDriveF.spin(vex::directionType::fwd,(TurnDriveMultiplyer * RC1Axis4),vex::velocityUnits::pct);
            LDriveB.spin(vex::directionType::rev,(TurnDriveMultiplyer * RC1Axis4),vex::velocityUnits::pct);
            RDriveF.spin(vex::directionType::rev,(TurnDriveMultiplyer * RC1Axis4),vex::velocityUnits::pct);
            RDriveB.spin(vex::directionType::fwd,(TurnDriveMultiplyer * RC1Axis4),vex::velocityUnits::pct);            
        }
        
        /// Point turns ///
        else if(abs(RC1Axis1) > Axis1Deadzone)
        {
            // Spin the motors in opposite directions
            LDriveF.spin(vex::directionType::fwd,(TurnDriveMultiplyer * RC1Axis1),vex::velocityUnits::pct);
            LDriveB.spin(vex::directionType::fwd,(TurnDriveMultiplyer * RC1Axis1),vex::velocityUnits::pct);
            RDriveF.spin(vex::directionType::rev,(TurnDriveMultiplyer * RC1Axis1),vex::velocityUnits::pct);
            RDriveB.spin(vex::directionType::rev,(TurnDriveMultiplyer * RC1Axis1),vex::velocityUnits::pct);
        }
       
        /// Straight driving ///
        else if (abs(RC1Axis3) > Axis3Deadzone) 
        {
            if(abs(RC1Axis3) >= FullDrivePowerThreshold) // If you push the controller all the way to 100, run at full speed
            {  
                //Spin the motors in the same direction at (full speed * speed multiplyer)
                LDriveF.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * RC1Axis3),vex::velocityUnits::pct);
                LDriveB.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * RC1Axis3),vex::velocityUnits::pct);
                RDriveF.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * RC1Axis3),vex::velocityUnits::pct);
                RDriveB.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * RC1Axis3),vex::velocityUnits::pct);
            }
            else //Otherwise, only go to half power
            {
                //Spin the motors in the same direction at (half speed * speed multiplyer)
                LDriveF.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * (RC1Axis3 * 0.5)),vex::velocityUnits::pct);
                LDriveB.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * (RC1Axis3 * 0.5)),vex::velocityUnits::pct);
                RDriveF.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * (RC1Axis3 * 0.5)),vex::velocityUnits::pct);
                RDriveB.spin(vex::directionType::fwd,(DrivePolarity * StraightDriveMultiplyer * (RC1Axis3 * 0.5)),vex::velocityUnits::pct);
            }   
        }
        
        /*
        //Lock wheels, uncomment when we can climb
        else if(RC1.ButtonUp.pressing() && !UpPressed) 
        {
            RDrive.stop(vex::brakeType::hold);
            LDrive.stop(vex::brakeType::hold);
        }  */

        else //If the left joystick is not being moved past the deadzone
        {
            LDriveF.stop();
            LDriveB.stop();
            RDriveF.stop();
            RDriveB.stop();
        }
        vex::task::sleep(20); 
    }
}

/////////////////////////////////////////////////////////////////////////////////////
//    _____                        _____                 _   _                     //
//   |_   _|____      _____ _ __  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___     //
//     | |/ _ \ \ /\ / / _ \ '__| | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|    //
//     | | (_) \ V  V /  __/ |    |  _|| |_| | | | | (__| |_| | (_) | | | \__ \    //
//     |_|\___/ \_/\_/ \___|_|    |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/    //
//                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////

///// Function to move tower up a certain amount based on encoder values /////
void MoveTowerByIncrement() 
{
    TowerMovingAutomatically = true;
    
    /// Setting tower speeds
    if(TowerDirection == 1) // if tower is moving upwards
    {
        TowerSpeed = 100;
        
        RTower.setStopping(vex::brakeType::hold);
        LTower.setStopping(vex::brakeType::hold);                        
    }
    else if(TowerDirection == -1) // if tower is moving down
    {
        TowerSpeed = 30;
        
        RTower.setStopping(vex::brakeType::brake);
        LTower.setStopping(vex::brakeType::brake);
    }
    
    /// Automatic movement of tower ///
    if(TowerCount == 0)
    {
        RTower.setStopping(vex::brakeType::coast);
        LTower.setStopping(vex::brakeType::coast);
        
        RTower.rotateTo(TowerBottomRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
        LTower.rotateTo(TowerBottomRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
    }
    if(TowerCount == 1)
    {
        RTower.setStopping(vex::brakeType::brake);
        LTower.setStopping(vex::brakeType::brake);
        
        RTower.rotateTo(LowHangStealRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
        LTower.rotateTo(LowHangStealRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
    }
    if(TowerCount == 2) 
    {
        RTower.setStopping(vex::brakeType::brake);
        LTower.setStopping(vex::brakeType::brake);
        
        RTower.rotateTo(LowHangPlaceRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
        LTower.rotateTo(LowHangPlaceRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
    }
    if(TowerCount == 3) 
    {
        RTower.setStopping(vex::brakeType::brake);
        LTower.setStopping(vex::brakeType::brake);
        
        TowerSpeed = 50;
        
        RTower.rotateTo(HighHangStealRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
        LTower.rotateTo(HighHangStealRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
    }
    if(TowerCount == 4) 
    {
        RTower.setStopping(vex::brakeType::brake);
        LTower.setStopping(vex::brakeType::brake);
        
        TowerSpeed = 40;
        
        RTower.rotateTo(TowerMaxRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
        LTower.rotateTo(TowerMaxRotation,vex::rotationUnits::deg,TowerSpeed,vex::velocityUnits::pct,false);
    }
}

void R1Pressed() 
{
    if(TowerCount < 4)
    {
        TowerCount++;
    }
    TowerDirection = 1;
    MoveTowerByIncrement();
}

void R2Pressed() 
{
    if(TowerCount > 0)
    {
        TowerCount--;
    }
    
    TowerDirection = -1;
    MoveTowerByIncrement();    
}

void ResetTowerTrack()
{
    RTower.resetRotation();
    LTower.resetRotation();
    TowerCount = 0;
}

void HoldTower()
{
    TowerHold = !TowerHold;
    if(TowerHold)
    {
        LTower.setStopping(vex::brakeType::hold);
        RTower.setStopping(vex::brakeType::hold);
    }
    else
    {
        LTower.setStopping(vex::brakeType::brake);
        RTower.setStopping(vex::brakeType::brake);
    }
}
int TowerControl() 
{    
    // Setup callbacks for various tower functions 
    RC2.ButtonR1.pressed(R1Pressed);
    RC2.ButtonR2.pressed(R2Pressed);
    
    RC2.ButtonA.pressed(HoldTower);
    RC2.ButtonX.pressed(ResetTowerTrack);
    
    // Manual tower control using right joystick on the partner controller
    while(1) 
    {
        int RC2Axis2 = RC2.Axis2.position(vex::percentUnits::pct);
        int TowerEncoder = RTower.rotation(vex::rotationUnits::deg);
        
        if(TowerEncoder > LowHangStealRotation) 
        {
            // Slow down the tower if it's too high
            //TowerSpeedMult = .5;
            TowerSpeedMult =1;
        } 
        else 
        {
            // Don't change the speed
            TowerSpeedMult = 1;
        }
        
        if(RC2Axis2 > TowerDeadzone)
        {
            // Move the tower up
            LTower.spin(vex::directionType::fwd,RC2Axis2 * TowerSpeedMult,vex::percentUnits::pct);
            RTower.spin(vex::directionType::fwd,RC2Axis2 * TowerSpeedMult,vex::percentUnits::pct);  
            TowerMovingAutomatically = false;
        }
        else if(RC2Axis2 < -TowerDeadzone)
        {
            // Move the tower down slower than moving up
            LTower.spin(vex::directionType::fwd,RC2Axis2 * 0.5 * TowerSpeedMult,vex::percentUnits::pct);
            RTower.spin(vex::directionType::fwd,RC2Axis2 * 0.5 * TowerSpeedMult,vex::percentUnits::pct);  
            TowerMovingAutomatically = false;
        }
        else 
        {
            if(!TowerMovingAutomatically && TowerDirection == -1)
            {
                LTower.stop();
                RTower.stop();
            }
            else if(!TowerMovingAutomatically)
            {
                LTower.stop();
                RTower.stop();
            }
        }
        vex::task::sleep(20);
    }
}


////////////////////////////////////////////////////////////////////////////////
//     ____ _                  _____                 _   _                    //
//    / ___| | __ ___      __ |  ___|   _ _ __   ___| |_(_) ___  _ __  ___    //
//   | |   | |/ _` \ \ /\ / / | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|   //
//   | |___| | (_| |\ V  V /  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \   //
//    \____|_|\__,_| \_/\_/   |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
void OpenClaw()
{
    ClawMovingAutomatically = true;
    Claw.rotateTo(ClawOpen,vex::rotationUnits::deg,ClawSpeed,vex::velocityUnits::pct,true);
    ClawMovingAutomatically = false;    
}

void CloseClaw()
{
    ClawMovingAutomatically = true;
    Claw.rotateTo(ClawClosed,vex::rotationUnits::deg,ClawSpeed,vex::velocityUnits::pct,true);
    ClawMovingAutomatically = false;    
}

void ResetClawEncoder() {
    Claw.resetRotation();
}
int ClawControl()
{
    // Functions to open and close the claw using a single button press
    RC2.ButtonLeft.pressed(OpenClaw);
    RC2.ButtonRight.pressed(CloseClaw);
    RC2.ButtonB.pressed(ResetClawEncoder); // Callback to reset the encoder
    // Manual claw control using L1 and L2 on the second controller
    while(1)
    {
        if(RC2.ButtonL1.pressing() || RC1.ButtonR1.pressing())
        {
           // Open the claw
           Claw.spin(vex::directionType::fwd, ClawSpeed, vex::velocityUnits::pct);
        }
        else if(RC2.ButtonL2.pressing() || RC1.ButtonR2.pressing())
        {
            // Close the claw
            Claw.spin(vex::directionType::rev, ClawSpeed, vex::velocityUnits::pct);
        }
        else
        {
            if(ClawMovingAutomatically == false)
            {
                // Hold the claw where it is
                Claw.stop(vex::brakeType::hold);
            }
        }
        vex::task::sleep(20); 
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//     ____ _                      _       _       _     _____                 _   _                   //
//    / ___| | __ ___      __     | | ___ (_)_ __ | |_  |  ____   _ _ __   ___| |_(_) ___  _ __  ___   //
//   | |   | |/ _` \ \ /\ / /  _  | |/ _ \| | '_ \| __| | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|  //
//   | |___| | (_| |\ V  V /  | |_| | (_) | | | | | |_  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \  //
//    \____|_|\__,_| \_/\_/    \___/ \___/|_|_| |_|\__| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/  //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Function to flip claw 180° /////
void FlipClawJoint() 
{
    ClawJointFlipped = !ClawJointFlipped;
    
    ClawJointMovingAutomatically = true;
    
    ClawJoint.setStopping(vex::brakeType::coast);
    ClawJoint.rotateTo(ClawJointFlipped * .5,vex::rotationUnits::rev,ClawJointSpeed,vex::velocityUnits::pct,false);
    vex::task::sleep(400);
    ClawJoint.rotateTo(ClawJointFlipped * .5,vex::rotationUnits::rev,ClawJointSpeed / 4,vex::velocityUnits::pct,true);
    ClawJointMovingAutomatically = false;
    ClawJoint.setStopping(vex::brakeType::hold);
}

void LiftAndFlipClawJoint() 
{
    RTower.setStopping(vex::brakeType::coast);
    LTower.setStopping(vex::brakeType::coast);
    
    TowerMovingAutomatically= true;
    
    RTower.startRotateTo(20,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    LTower.startRotateTo(20,vex::rotationUnits::deg,100,vex::velocityUnits::pct);
    FlipClawJoint();
}
void ResetClawJointEncoder() {
    ClawJoint.resetRotation();
}
int ClawJointControl()
{
    RC2.ButtonUp.pressed(LiftAndFlipClawJoint); // Callback to lift tower and flip claw
    RC2.ButtonDown.pressed(FlipClawJoint); // Callback to flip the claw when 
    RC2.ButtonB.pressed(ResetClawJointEncoder); // Callback to reset the encoder
    
    while(1)
    {
       // Manual override for claw joint
       int RC2Axis4 = RC2.Axis4.position(vex::percentUnits::pct);
       
       if(!ClawJointMovingAutomatically)
       {
           if(abs(RC2Axis4) > ClawJointDeadzone)
           {
               ClawJoint.spin(vex::directionType::fwd,RC2Axis4 * ClawJointManualMultiplyer,vex::velocityUnits::pct);
           }
           else
           {
               ClawJoint.stop(vex::brakeType::hold);
           }
       }
        vex::task::sleep(20);
    }  
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//    _                           _                 _____                 _   _                     //
//   | |    __ _ _   _ _ __   ___| |__   ___ _ __  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___     //
//   | |   / _` | | | | '_ \ / __| '_ \ / _ \ '__| | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|    //
//   | |__| (_| | |_| | | | | (__| | | |  __/ |    |  _|| |_| | | | | (__| |_| | (_) | | | \__ \    //
//   |_____\__,_|\__,_|_| |_|\___|_| |_|\___|_|    |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/    //
//                                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////////////////////
///// We have no launcher :( /////
/*
///// Function to Cock/Fire Launcher /////
void CockLauncher() 
{
    LauncherMovingAutomatically = true;
    
    if(Cocked) //Fire the launcher if it's cocked
    {
        RC1.rumble("-");
        Launcher.rotateTo(8.4,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        LauncherMovingAutomatically = false;
    }
    
    else //Otherwise, cock the launcher
    {
        Launcher.setRotation(0,vex::rotationUnits::rev);
        RC1.rumble(".");
        Launcher.rotateTo(5.,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        LauncherMovingAutomatically = false;
    }
    Cocked = !Cocked; //Change the state of the launcher after it has cocked/fired
}

///// Main function for controlling launcher /////
int LauncherControl()
{
    //RC1.ButtonLeft.pressed(CockLauncher);
    Launcher.setStopping(vex::brakeType::brake);
    while(1) {
        int RC2Axis1 = RC2.Axis1.position(vex::percentUnits::pct); 
        
        if(RC2Axis1 > 50 || RC2Axis1 < -50) 
        {
            Launcher.spin(vex::directionType::rev,RC2Axis1,vex::velocityUnits::pct);
        }
        else if(!LauncherMovingAutomatically) {
            Launcher.stop();
        }
        
        else if(RC1.ButtonLeft.pressing()) {
            Launcher.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        }
        else {
            Launcher.stop(vex::brakeType::hold);
        }
       vex::task::sleep(20); 
    }
}
*/
    

//////////////////////////////////////////////////////////////////////////////////////
//    ___       _        _          _____                 _   _                     //
//   |_ _|_ __ | |_ __ _| | _____  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___     //
//    | || '_ \| __/ _` | |/ / _ \ | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|    //
//    | || | | | || (_| |   <  __/ |  _|| |_| | | | | (__| |_| | (_) | | | \__ \    //
//   |___|_| |_|\__\__,_|_|\_\___| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/    //
//                                                                                  //
//////////////////////////////////////////////////////////////////////////////////////
///// No Intake because no launcher :( /////
/*
void ToggleIntake() 
{  
    if(!IntakeToggled) 
    {
        if(!Cocked)
        {
            CockLauncher();
        }
        //AimerMovingAutomatically = true;
        Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        //Aimer.rotateTo(-750,vex::rotationUnits::deg,100,vex::velocityUnits::pct,true);
        //AimerMovingAutomatically = false;
    }
    else 
    { 
        Intake.stop(vex::brakeType::coast);
    }
    
    IntakeToggled = !IntakeToggled;
}

int IntakeControl() {
    Intake.setStopping(vex::brakeType::brake);
    RC1.ButtonUp.pressed(ToggleIntake);
    
    while(1)
    {
        vex::task::sleep(20);
    }
}
*/
    
//////////////////////////////////////////////////////////////////////////////////////////
//     ____            _             _ _             ____                               //
//    / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __  / ___|  ___ _ __ ___  ___ _ __      //
//   | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__| \___ \ / __| '__/ _ \/ _ \ '_ \     //
//   | |__| (_) | | | | |_| | | (_) | | |  __/ |     ___) | (__| | |  __/  __/ | | |    //
//    \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|    |____/ \___|_|  \___|\___|_| |_|    //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////

//// Clear the screen at a specific x and y location /////
void ControllerClearScreenAt(vex::controller rc, int x, int y)
{
    rc.Screen.setCursor(y,x);
    rc.Screen.print(" ");
}

///// Overloaded function to clear part of a line /////
void ControllerClearScreenAt(vex::controller rc, int x1, int x2, int y)
{
    int count = x1;
    while(count <= x2)
    {
        ControllerClearScreenAt(rc, count, y);
        count++;
    }
}

///// Print a string at a specific x and y location /////
void ControllerPrintScreenAt(vex::controller rc, int x, int y, std::string input)
{
    rc.Screen.setCursor(y,x);
    rc.Screen.print(input.c_str());
}

///// Print an int at a specific x and y location /////
void ControllerPrintScreenAt(vex::controller rc, int x, int y, int input)
{
    rc.Screen.setCursor(y,x);
    rc.Screen.print(input);
}

///// Print a double at a specific x and y location /////
void ControllerPrintScreenAt(vex::controller rc, int x, int y, double input)
{
    rc.Screen.setCursor(y,x);
    rc.Screen.print(input);
}

///// Print a float at a specific x and y location /////
void ControllerPrintScreenAt(vex::controller rc, int x, int y, float input)
{
    rc.Screen.setCursor(y,x);
    rc.Screen.print(input);
}

///// Function to print various types of information on the controller
void ControllerScreenInfo(vex::controller rc, int x, int y, std::string info)
{
    if(info == "drive")
    {
        ControllerPrintScreenAt(rc,x,y,StraightDriveMultiplyer);
        //ControllerPrintScreenAt(rc,)
    }
    else if(info == "towerencoder")
    {
        //Prints the encoder value of RTower motor
        ControllerPrintScreenAt(rc,x,y,RTower.rotation(vex::rotationUnits::deg));
    }
    else if(info == "towercount")
    {
        ControllerPrintScreenAt(rc,x,y,TowerCount);
    }
    else if(info == "launcher")
    {
        //TODO
    }
    else if(info == "claw")
    {
        if(!ClawJointFlipped)
        {
            ControllerPrintScreenAt(rc,x,y,"Pick Up");
        }
        else
        {
            ControllerPrintScreenAt(rc,x,y,"Hang   ");
        }
    }
    else if(info == "leftsonar")
    {
        ControllerPrintScreenAt(rc,x,y,SonarLeft.distance(vex::distanceUnits::cm));
    }
    else if(info == "rightsonar")
    {
        ControllerPrintScreenAt(rc,x,y,SonarRight.distance(vex::distanceUnits::cm));
    }
}

///// Function that controlls what's on the screen /////
int ControllerScreen()
{
    // First controller initialization    
    RC1.Screen.clearScreen();
    ControllerPrintScreenAt(RC1,1,1,"Drive:");
    
    // Second controller screen    
    RC2.Screen.clearScreen();
    ControllerPrintScreenAt(RC2,1,1,"TwrCnt:");
    ControllerPrintScreenAt(RC2,1,2,"Claw:");
    ControllerPrintScreenAt(RC2,1,3,"Tower:");
    
    while(1)
    {
        // Primary controller info to be printed continuously
        ControllerScreenInfo(RC1,8,1,"drive");
        
        // Secondary controller info to be printed continuously
        ControllerScreenInfo(RC2,9,1,"towercount");
        ControllerScreenInfo(RC2,7,2,"claw");
        ControllerScreenInfo(RC2,8,3,"towerencoder");
        vex::task::sleep(20);
    }
}
///////////////////////////////////////////////////////////////////
//    ____            _         ____                             //
//   | __ ) _ __ __ _(_)_ __   / ___|  ___ _ __ ___  ___ _ __    //
//   |  _ \| '__/ _` | | '_ \  \___ \ / __| '__/ _ \/ _ \ '_ \   //
//   | |_) | | | (_| | | | | |  ___) | (__| | |  __/  __/ | | |  //
//   |____/|_|  \__,_|_|_| |_| |____/ \___|_|  \___|\___|_| |_|  //
//                                                               //
///////////////////////////////////////////////////////////////////
class Button 
{
    public:
        // Dimensions of the button
        int StartX;
        int StartY;
        int Width;
        int Height;
        int EndX;
        int EndY;

        std::function<void(void)> ReleaseFunction; // Function to be called when the button is pressed (released)
    
    public:
        ///// Constructors to create and draw a button on the brain /////
        Button ();
        Button (int, int, int, int, std::function<void(void)>);
        Button (int, int, int, int, int, std::function<void(void)>);
        Button (int, int, int, int, std::string, std::function<void(void)>);
        Button (int, int, int, int, vex::color, std::function<void(void)>);
    
        void GetRelease()
        {
            return ReleaseFunction();
        }

        //Doesn't work for some reason
        void SetRelease(std::function<void(void)> inputFunc)
        {
            ReleaseFunction = inputFunc;
        }
    
        ///// Function to change the color of a button
        void setColor(std::string c)
        {
            Cerebro.Screen.setPenColor(c.c_str()); // Set the outline color of the rectangle
            Cerebro.Screen.drawRectangle(StartX, StartY, Width, Height, c.c_str()); // Draw a rectangle in the same place as the old button
            Cerebro.Screen.setPenColor("#FFFFFF"); // Set the pen color back to white
        }
};

std::vector<Button> Buttons; // Create a vector to store all the buttons

///// Function to create a button /////
Button::Button(int x, int y, int w, int h, std::function<void(void)> inputFunc)
{
    // Set the x dimensions
    StartX = x;
    Width = w;
    EndX = StartX + Width;

    // Set the y dimesnsions
    StartY = y;
    Height = h;
    EndY = StartY + Height;
    
    // Set the release function
    ReleaseFunction = inputFunc;
    
    Cerebro.Screen.setPenColor("#FFFFFF"); // Set the outline color to white
    Cerebro.Screen.drawRectangle(StartX, StartY, Width, Height,"#FFFFFF"); //Draw the button
    
    Buttons.push_back(*this); //Add the button to the vector
}

///// Create a button with an int color value /////
Button::Button(int x, int y, int w, int h, int c, std::function<void(void)> inputFunc)
{
    // Set the x dimensions
    StartX = x;
    Width = w;
    EndX = StartX + Width;

    // Set the y dimesnsions
    StartY = y;
    Height = h;
    EndY = StartY + Height;

    ReleaseFunction = inputFunc; // Set the release function
    
    Cerebro.Screen.setPenColor(c);  // Set the border color to the same as the rest of the button
    Cerebro.Screen.drawRectangle(StartX, StartY, Width, Height, c); //Draw the button
    Cerebro.Screen.setPenColor("#FFFFFF"); //Set the pen color back to white    

    Buttons.push_back(*this); //Add the button to the vector
}

///// Create a button with a hexadecimal color /////
Button::Button(int x, int y, int w, int h, std::string c, std::function<void(void)> inputFunc)
{
    // Set the x dimensions
    StartX = x;
    Width = w;
    EndX = StartX + Width;

    // Set the y dimesnsions
    StartY = y;
    Height = h;
    EndY = StartY + Height;
    
    // Set the release function
    ReleaseFunction = inputFunc;
    
    Cerebro.Screen.setPenColor(c.c_str()); // Set the border color to the same as the rest of the button
    Cerebro.Screen.drawRectangle(StartX, StartY, Width, Height, c.c_str()); // Draw the button
    Cerebro.Screen.setPenColor("#FFFFFF"); // Set the pen color back to white

    Buttons.push_back(*this); // Add the button to the vector
}

///// Create a button with a vex::color color /////
Button::Button(int x, int y, int w, int h, vex::color c, std::function<void(void)> inputFunc)
{
    // Set the x dimensions
    StartX = x;
    Width = w;
    EndX = StartX + Width;

    // Set the y dimesnsions
    StartY = y;
    Height = h;
    EndY = StartY + Height;
    
    ReleaseFunction = inputFunc; // Set the release function
    
    Cerebro.Screen.setPenColor(c); // Set the border color to the same as the rest of the button
    Cerebro.Screen.drawRectangle(StartX, StartY, Width, Height, c); // Draw the button
    Cerebro.Screen.setPenColor("#FFFFFF"); // Set the pen color back to white

    Buttons.push_back(*this); // Add the button to the vector
}

void ButtonPress()
{
    // Get the location of the screen press
    int PressX = Cerebro.Screen.xPosition();
    int PressY = Cerebro.Screen.yPosition();
    
    for(int i = 0; i < Buttons.size(); i++)
    {
        Button B = Buttons.at(i);
        
        // If the button is pressed on
        if(PressX >  B.StartX && PressX < B.EndX && PressY > B.StartY && PressY < B.EndY)
        {      
            // Store the location of the button that was pressed so it can be deleted when ButtonRelease() is called
            PressButtonStartX = B.StartX;
            PressButtonStartY = B.StartY;
            PressButtonEndX = B.EndX;
            PressButtonEndY = B.EndY;
            
            // Create a visual indicator of the button press
            Cerebro.Screen.setPenColor("#FFFFFF");
            Cerebro.Screen.drawLine(B.StartX - 1, B.StartY - 1, B.EndX + 1, B.StartY - 1); // Top border
            Cerebro.Screen.drawLine(B.StartX - 1, B.EndY + 1, B.EndX + 1, B.EndY + 1); // Bottom border
            Cerebro.Screen.drawLine(B.StartX - 1, B.StartY - 1, B.StartX - 1, B.EndY + 1); // Left border
            Cerebro.Screen.drawLine(B.EndX + 1, B.StartY - 1, B.EndX + 1, B.EndY - 1); // Right border
        }
    }
}

void ButtonRelease()
{
    // Get the location of the screen press
    int PressX = Cerebro.Screen.xPosition();
    int PressY = Cerebro.Screen.yPosition();
    
    // Remove the visual indicator of the last button that was pressed
    Cerebro.Screen.setPenColor("#000000");
    Cerebro.Screen.drawLine(PressButtonStartX - 1, PressButtonStartY - 1, PressButtonEndX + 1, PressButtonStartY - 1); // Top border
    Cerebro.Screen.drawLine(PressButtonStartX - 1, PressButtonEndY + 1, PressButtonEndX + 1, PressButtonEndY + 1); // Bottom border
    Cerebro.Screen.drawLine(PressButtonStartX - 1, PressButtonStartY - 1, PressButtonStartX - 1, PressButtonEndY + 1); // Left border
    Cerebro.Screen.drawLine(PressButtonEndX + 1, PressButtonStartY - 1, PressButtonEndX + 1, PressButtonEndY - 1); // Right border
    Cerebro.Screen.setPenColor("FFFFFF");
    
    for(int i = 0; i < Buttons.size(); i++)
    {        
        // If you release the screen on top of the button
        if(PressX >  Buttons.at(i).StartX && PressX < Buttons.at(i).EndX && PressY > Buttons.at(i).StartY && PressY < Buttons.at(i).EndY)
        {
            Buttons.at(i).GetRelease(); // Run the callback function
        }
    }
}

///// Function to remove all buttons and clear the brain's screen and resets the cursor and color /////
void ClearBrainScreen()
{
    Buttons.clear();
    Cerebro.Screen.clearScreen();
    Cerebro.Screen.setPenColor("#FFFFFF");
    Cerebro.Screen.setCursor(1,1);
}

void BlueCloseAuton()
{
    isBlue = true;
    isClose = true;
    Cerebro.Screen.setPenColor("#0000FF");
    Cerebro.Screen.printAt(100,15,"Selected Blue Close         ");
}

void BlueFarAuton()
{
    isBlue = true;
    isClose = false;
    isAny = false;
    Cerebro.Screen.setPenColor("#0000FF");
    Cerebro.Screen.printAt(100,15,"Selected Blue Far          ");
}

void RedCloseAuton()
{
    isBlue = false;
    isClose = true;
    isAny = false;
    Cerebro.Screen.setPenColor("#FF0000");
    Cerebro.Screen.printAt(100,15,"Selected Red Close         ");
}

void RedFarAuton()
{
    isBlue = false;
    isClose = false;
    isAny = false;
    Cerebro.Screen.setPenColor("#FF0000");
    Cerebro.Screen.printAt(100,15,"Selected Red Far           ");
}

void AnyAuton()
{
    isAny = true; 
    Cerebro.Screen.setPenColor("#00FF00");
    Cerebro.Screen.printAt(100,15,"Selected any autonomous");
}

void BrainAutonScreen()
{
    Cerebro.Screen.printAt(20,45,"Blue Close");
    Button btnBlueFront(50,100,75,75, "#0000FF", BlueCloseAuton);
    
    Cerebro.Screen.printAt(150,45,"Blue Far");
    Button btnBlueBack(150,100,75,75, "#0000FF", BlueFarAuton);
    
    Cerebro.Screen.printAt(250,45,"Red Close");
    Button btnRedFront(250,100,75,75, "#FF0000", RedCloseAuton);
    
    Cerebro.Screen.printAt(350,45,"Red Far");
    Button btnRedBack(350,100,75,75, "#FF0000", RedFarAuton);
    
    Cerebro.Screen.printAt(175,200,"Anywhere");
    Button btnAny(175,200,100,25,"#00FF00", AnyAuton);
}

///// The most important function in the whole program /////
int HailSatan()
{
    Cerebro.Screen.clearScreen();
    Cerebro.Screen.setFont(vex::fontType::mono30);
    Cerebro.Screen.setPenColor("#cc0000");
    while(1)
    {
        Cerebro.Screen.clearScreen();
        Cerebro.Screen.setCursor(1,6);
        Cerebro.Screen.setPenColor("#e6e600");
        Cerebro.Screen.print("    )        (   (     ");
        Cerebro.Screen.setCursor(2,6);
        Cerebro.Screen.print(" ( /(  (     )\\ ))\\ )  ");
        Cerebro.Screen.setCursor(3,6);
        Cerebro.Screen.setPenColor("#cc6600");
        Cerebro.Screen.print(" )\\()) )\\   (()/(()/(  ");
        Cerebro.Screen.setCursor(4,6);
        Cerebro.Screen.setPenColor("#d74b00");
        Cerebro.Screen.print("((_)((((_)(  /(_))(_)) ");
        Cerebro.Screen.setCursor(5,6);
        Cerebro.Screen.setPenColor("#d74600");
        Cerebro.Screen.print(" _((_)\\ _ )\\(_))(_))   ");
        Cerebro.Screen.setCursor(6,6);
        Cerebro.Screen.setPenColor("#d70000");
        Cerebro.Screen.print("| || (_)_\\(_)_ _| |    ");
        Cerebro.Screen.setCursor(7,6);
        Cerebro.Screen.print("| __ |/ _ \\  | || |__  ");
        Cerebro.Screen.setCursor(8,6);
        Cerebro.Screen.print("|_||_/_/ \\_\\|___|____| ");
        vex::task::sleep(1500);
        Cerebro.Screen.clearScreen();
        
        Cerebro.Screen.setCursor(1,1);
        Cerebro.Screen.setPenColor("#e6e600");
        Cerebro.Screen.print(" (                            )  ");
        Cerebro.Screen.setCursor(2,1);
        Cerebro.Screen.print(" )\\ )   (      *   )  (    ( /(  ");
        Cerebro.Screen.setCursor(3,1);
        Cerebro.Screen.setPenColor("#cc6600");
        Cerebro.Screen.print("(()/(   )\\   ` )  /(  )\\    )\\()) ");
        Cerebro.Screen.setCursor(4,1);
        Cerebro.Screen.setPenColor("#d75a00"); 
        Cerebro.Screen.print(" /(_)((((_)(  ( )(_) (_)( ((_)\\  ");
        Cerebro.Screen.setCursor(5,1);
        Cerebro.Screen.setPenColor("#d72800");
        Cerebro.Screen.print("(_))  )\\ _ )\\(_(_())\\ _ )\\ _((_) ");
        Cerebro.Screen.setCursor(6,1);
        Cerebro.Screen.setPenColor("#d70000");
        Cerebro.Screen.print("/ __| (_)_\\(_|_   _| /_\\(_| \\| | ");
        Cerebro.Screen.setCursor(7,1);
        Cerebro.Screen.print("\\__ \\  / _ \\   | |  / _ \\ | .` | ");
        Cerebro.Screen.setCursor(8,1);
        Cerebro.Screen.print("|___/ /_/ \\_\\  |_| /_/ \\_\\|_|\\_| ");
        vex::task::sleep(1500);
    }
}
//////////////////////////////////////////////////////////////////////////////////////////
//    ____                   _         _                                                //
//   |  _ \ _ __ ___        / \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___   //
//   | |_) | '__/ _ \_____ / _ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __|  //
//   |  __/| | |  __|_____/ ___ | |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \  //
//   |_|   |_|  \___|    /_/   \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/  //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
void pre_auton( void ) 
{
    Cerebro.Screen.pressed(ButtonPress); //Set up the button indicatior callback
    Cerebro.Screen.released(ButtonRelease); // Set up the screen press (release) callback
    BrainAutonScreen(); // Startup the autonomous program selector
}
///////////////////////////////////////////////////////////////////////
//                  _                                                //
//       /\        | |                                               //
//      /  \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___   //
//     / /\ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __|  //
//    / ____ \ |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \  //
//   /_/    \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/  //
//                                                                   //
///////////////////////////////////////////////////////////////////////
void autonomous( void ) 
{
    // reset encoders, set stopping defaults, 
    RTower.resetRotation();
    LTower.resetRotation();
    LDriveF.resetRotation();
    LDriveB.resetRotation();
    RDriveF.resetRotation();
    RDriveB.resetRotation();
    Claw.resetRotation();
    ClawJoint.resetRotation();
    ClawJoint.setStopping(vex::brakeType::hold);
    Claw.setStopping(vex::brakeType::hold);
    LTower.setStopping(vex::brakeType::hold);
    RTower.setStopping(vex::brakeType::hold);
    LDriveF.setStopping(vex::brakeType::hold);
    LDriveB.setStopping(vex::brakeType::hold);
    RDriveF.setStopping(vex::brakeType::hold);
    RDriveB.setStopping(vex::brakeType::hold);
    LDriveF.stop();
    LDriveB.stop();
    RDriveF.stop();
    RDriveB.stop();
    Claw.stop();
    ClawJoint.stop();
    
    ////////////////// BLUE CLOSE && RED FAR AUTON ////////////
    if((isBlue == true && isClose == true && isAny == false) || (isBlue == false && isClose == false && isAny == false))
    {
        Cerebro.Screen.clearScreen();
        Cerebro.Screen.printAt(50,50,"Blue Close auton");
        ///////////////////////////////////////// PART 1 ///////////////////////////////////////////////
        //close claw, therefore having it fold out
        Claw.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
        vex::task::sleep(1000);
        //Claw.setMaxTorque(100,vex::percentUnits::pct);
        Claw.stop();
        Claw.resetRotation();
    
        // rotate claw to pick up rotation
        ClawJoint.rotateFor(-.25,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        ClawJoint.setRotation(0,vex::rotationUnits::rev);
    
        // lower tower and reset encoder at bottom    
        RTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        LTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        vex::task::sleep(500);
        RTower.resetRotation();
        LTower.resetRotation();
        RTower.stop();
        LTower.stop();
    
        // Raise Tower Slightly
        RTower.rotateFor(200,vex::rotationUnits::deg,100,vex::velocityUnits::pct,false);
        
        // drive forward & open claw (knock cap off of ball)
        LDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        LDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,true);
    
         // back up
        LDriveF.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        
        // turn left 90 degrees
        LDriveF.rotateFor(-1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(-1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,true);
    
        // drive backwards to platform
        LDriveF.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        LDriveB.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        RDriveF.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        RDriveB.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        vex::task::sleep(1400);
        LDriveF.stop();
        LDriveB.stop();
        RDriveF.stop();
        RDriveB.stop();
    }
    
    ////////////////// RED CLOSE && BLUE FAR AUTON ////////////
    //if((isBlue == false && isClose == true && isAny == false) || (isBlue == true && isClose == false && isAny == false))
    
    // blue far 
    if(isBlue == true && isClose == false && isAny == false) 
    {
        Cerebro.Screen.clearScreen();
        Cerebro.Screen.printAt(50,50,"Blue Close auton");
        ///////////////////////////////////////// PART 1 ///////////////////////////////////////////////
        //close claw, therefore having it fold out
        Claw.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
        vex::task::sleep(1000);
        //Claw.setMaxTorque(100,vex::percentUnits::pct);
        Claw.stop();
        Claw.resetRotation();
    
        // rotate claw to pick up rotation
        ClawJoint.rotateFor(-.25,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        ClawJoint.setRotation(0,vex::rotationUnits::rev);
    
        // lower tower and reset encoder at bottom    
        RTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        LTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        vex::task::sleep(500);
        RTower.resetRotation();
        LTower.resetRotation();
        RTower.stop();
        LTower.stop();
        
        // Raise Tower Slightly
        RTower.rotateFor(200,vex::rotationUnits::deg,100,vex::velocityUnits::pct,false);
        
        // drive forward & open claw (knock cap off of ball)
        LDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        LDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,true);
    
         // back up
        LDriveF.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        
        // turn right 90 degrees
        LDriveF.rotateFor(1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(-1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(-1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,true);
    
        // drive backwards to platform
        LDriveF.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        LDriveB.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        RDriveF.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        RDriveB.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        vex::task::sleep(1400);
        LDriveF.stop();
        LDriveB.stop();
        RDriveF.stop();
        RDriveB.stop();
    }
    
    // experimental red close
    if(isBlue == false && isClose == true && isAny == false)
    {
        Cerebro.Screen.clearScreen();
        Cerebro.Screen.printAt(50,50,"Blue Close auton");
        //close claw, therefore having it fold out
        Claw.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
        vex::task::sleep(1000);
        //Claw.setMaxTorque(100,vex::percentUnits::pct);
        Claw.stop();
        Claw.resetRotation();
    
        // rotate claw to pick up rotation
        ClawJoint.rotateFor(-.25,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        ClawJoint.setRotation(0,vex::rotationUnits::rev);
    
        // lower tower and reset encoder at bottom    
        RTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        LTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        vex::task::sleep(500);
        RTower.resetRotation();
        LTower.resetRotation();
        RTower.stop();
        LTower.stop();
        ///////////////////////////////////////// Go for flag ///////////////////////////////////////////////
        // drive backwards (toggle low flag)
        LDriveF.rotateTo(-3.6,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        LDriveB.rotateTo(-3.6,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveF.rotateTo(-3.6,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveB.rotateTo(-3.6,vex::rotationUnits::rev,80,vex::velocityUnits::pct,true);
        
        // drive forwards (return to starting tile)
        LDriveF.rotateFor(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,true);
        
        // turn left 90 degrees
        LDriveF.rotateFor(-1.9,vex::rotationUnits::rev,70,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(-1.9,vex::rotationUnits::rev,70,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(1.9,vex::rotationUnits::rev,70,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(1.9,vex::rotationUnits::rev,70,vex::velocityUnits::pct,true);
        ///////////////////////////////////////// Knock off cap and park ///////////////////////////////////////////////
        
        
         // Raise Tower Slightly
        RTower.rotateFor(200,vex::rotationUnits::deg,100,vex::velocityUnits::pct,false);
    
        // drive forward (knock cap off of ball)
        LDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        LDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,true);
    
         // back up
        LDriveF.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(-.2,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        
        // turn right 90 degrees
        LDriveF.rotateFor(1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(-1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,false);
        RDriveB.rotateFor(-1.9,vex::rotationUnits::rev,50,vex::velocityUnits::pct,true);
    
        // drive backwards to platform
        LDriveF.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        LDriveB.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        RDriveF.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        RDriveB.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        vex::task::sleep(1400);
        LDriveF.stop();
        LDriveB.stop();
        RDriveF.stop();
        RDriveB.stop();
    }
    
    
    if(isAny == true)
    {
        Cerebro.Screen.clearScreen();
        Cerebro.Screen.printAt(50,50,"Any auton");
        ///////////////////////////////////////// PART 1 ///////////////////////////////////////////////
        //close claw, therefore having it fold out
        Claw.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
        vex::task::sleep(1000);
        //Claw.setMaxTorque(100,vex::percentUnits::pct);
        Claw.stop();
        Claw.resetRotation();
    
        // rotate claw to pick up rotation
        ClawJoint.rotateFor(-.25,vex::rotationUnits::rev,100,vex::velocityUnits::pct,true);
        ClawJoint.setRotation(0,vex::rotationUnits::rev);
    
        // lower tower and reset encoder at bottom    
        RTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        LTower.spin(vex::directionType::rev,100,vex::percentUnits::pct);
        vex::task::sleep(1000);
        RTower.resetRotation();
        LTower.resetRotation();
        RTower.stop();
        LTower.stop();
    
        // drive forward & open claw (knock cap off of ball)
        LDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        LDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveF.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,false);
        RDriveB.rotateTo(3.4,vex::rotationUnits::rev,80,vex::velocityUnits::pct,true);
        
        // Drive backwards to not touch cap at end of auton
        LDriveF.rotateFor(-.5,vex::rotationUnits::rev,40,vex::velocityUnits::pct,false);
        LDriveB.rotateFor(-.5,vex::rotationUnits::rev,40,vex::velocityUnits::pct,false);
        RDriveF.rotateFor(-.5,vex::rotationUnits::rev,40,vex::velocityUnits::pct,false);    
        RDriveB.rotateFor(-.5,vex::rotationUnits::rev,40,vex::velocityUnits::pct,true);    
    }
}


///////////////////////////////////////////////////////////////////////////
//    _____       _                   _____            _             _   //
//   |  __ \     (_)                 / ____|          | |           | |  //
//   | |  | |_ __ ___   _____ _ __  | |     ___  _ __ | |_ _ __ ___ | |  //
//   | |  | | '__| \ \ / / _ \ '__| | |    / _ \| '_ \| __| '__/ _ \| |  //
//   | |__| | |  | |\ V /  __/ |    | |___| (_) | | | | |_| | | (_) | |  //
//   |_____/|_|  |_| \_/ \___|_|     \_____\___/|_| |_|\__|_|  \___/|_|  //
//                                                                       //
///////////////////////////////////////////////////////////////////////////
void usercontrol( void )
{
    LDriveF.setStopping(vex::brakeType::coast);
    LDriveB.setStopping(vex::brakeType::coast);
    RDriveF.setStopping(vex::brakeType::coast);
    RDriveB.setStopping(vex::brakeType::coast);
    Claw.resetRotation();
    
    ///// Start driver tasks /////
    vex::task TC(TowerControl);  
    vex::task CC(ClawControl);
    vex::task JC(ClawJointControl);
    vex::task DC(DriveControl);
    //vex::task LC(LauncherControl); RIP lawn chair
    //vex::task IC(IntakeControl); RIP
    vex::task CS(ControllerScreen);    
    vex::task HS(HailSatan);    
    vex::task::sleep(20);
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
    //Run the pre-autonomous function. 
    pre_auton();
    
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) 
    {
        vex::task::sleep(100); //Sleep the task for a short amount of time to prevent wasted resources.
    }        
}
