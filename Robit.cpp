/*********************************************************************************//**
* @file Robit.cpp
* Source file for the myRobit class and its thread functions
*************************************************************************************/

/*************************************************************************************
* Includes
*************************************************************************************/
#include "Robit.h"

/*************************************************************************************
* Global variable definitions
*************************************************************************************/
float               desVoltage = 290;       /**< Desired voltage on load cell       */
bool                driveRun = true;        /**< Whether drive should be running    */
bool                winding = false;        /**< Keeps track of the state of the winch */
int                 ballAqState = 0;        /**< Command to the ball aq thread motors */
                                            /* 1 = Forwards, -1 = Reverse, 0 = DC   */
int                 ballAqPistonState = 0;  /**< Command to the ball aq thread pistons */
                                            /* 1 = Out, -1 = In, 0 = Neutral, 2 = DC */

/*********************************************************************************//**
* myRobit class constructor
*************************************************************************************/
myRobit::myRobit(){};

/*************************************************************************************
* myRobit class destructor
*************************************************************************************/
myRobit::~myRobit(){};

/*************************************************************************************
Unused class functions
*************************************************************************************/
void myRobit::TestInit(){};

void myRobit::DisabledPeriodic(){};
void myRobit::AutonomousPeriodic(){};
void myRobit::TeleopPeriodic(){};
void myRobit::TestPeriodic(){};

/*********************************************************************************//**
* Robot init function
*
* Runs after all OS setup. Sets up global variables for all the mechanisms, then
* spawns all the worker threads.
*************************************************************************************/
void myRobit::RobotInit(){
    // Set up the drive object
    rDrive =                new RobotDrive( LF_DRIVE_PORT, LR_DRIVE_PORT, RF_DRIVE_PORT, RR_DRIVE_PORT );

    // Set up the control objects
    ACDC =                  new AnalogChannel( LOADCELL_CHANNEL );
    aqDeploy =              new DoubleSolenoid( BALLAQ_DEPLOY, BALLAQ_UNDEPLOY );
    aqDeploy2 =             new DoubleSolenoid( BALLAQ2_DEPLOY, BALLAQ2_UNDEPLOY );
    compressor =            new Compressor( PRESSURE_SWITCH, COMPRESSOR_PORT );
    filthyWench =           new Talon( WINCH_PORT );
    joystick =              new Joystick( JOY_PORT_1 );
    lcd =                   DriverStationLCD::GetInstance( );
    limitSwitch =           new DigitalInput( SWITCH_PORT );
    poleRaiser =            new Talon( DEFENSE_MOTOR );
    rangeFinder =           new AnalogChannel( RANGE_SENSOR );
    winchRelease =          new DoubleSolenoid( WINCH_RELEASE_PORT, WINCH_RELEASE_OTHER );
    windInBall =            new Talon( BALLAQ_WINDIN );

    // Set all starting values for objects
    aqDeploy->Set( DoubleSolenoid::kReverse );
    aqDeploy2->Set( DoubleSolenoid::kReverse );
    lcd->PrintfLine(DriverStationLCD::kUser_Line2, "%.0f", desVoltage );
    rDrive->SetSafetyEnabled( false );
    winchRelease->Set( DoubleSolenoid::kReverse );

    // Start all the threads
    pthread_create( &driveThread, NULL, driveFunc, NULL );
    pthread_create( &inputThread, NULL, inputFunc, NULL );
    pthread_create( &winchThread, NULL, winchFunc, NULL );
}

/*********************************************************************************//**
* Autonomous init function
*
* Runs once at start of autonomous period.
*************************************************************************************/
void myRobit::AutonomousInit(){
    driveRun = false;
    Wait( .02 );

    // Drop the lightsaber
    poleRaiser->Set(DoubleSolenoid::kForward);

    // Cock the catapult

    // Move into position
    rDrive->MecanumDrive_Cartesian(0, 1, 0);
    Wait( 0.5 );
    rDrive->MecanumDrive_Cartesian(0, .5, 0);
    Wait( 0.5 );
    rDrive->MecanumDrive_Cartesian(0, 0, 0);

    // fire
    winchRelease->Set( DoubleSolenoid::kForward );
    Wait( 2.5 );
    winchRelease->Set( DoubleSolenoid::kReverse );
}

/*********************************************************************************//**
* Teleop init function
*
* Runs once at start of teleop period.
*************************************************************************************/
void myRobit::TeleopInit(){
    // Start the compressor
    compressor->Start( );

    driveRun = true;
    ballAqState = 0;
    ballAqPistonState = 2;
}

/*********************************************************************************//**
* Disabled init function
*
* Runs once at start of disabled period.
*************************************************************************************/
void myRobit::DisabledInit(){
    // Stop the compressor
    compressor->Stop( );
}

/*********************************************************************************//**
* Drive thread function
*
* Drives the robot concurrently with other actuators
*************************************************************************************/
void * driveFunc( void * arg ){

    while ( 1 ){
        // Get the input values
        float tX = ( fabs( joystick->GetRawAxis( JOY_AXIS_LX ) ) > DEAD_ZONE ) ? joystick->GetRawAxis( JOY_AXIS_LX ) : 0.0;
        float tY = ( fabs( joystick->GetRawAxis( JOY_AXIS_LY ) ) > DEAD_ZONE ) ? joystick->GetRawAxis( JOY_AXIS_LY ) : 0.0;
        float tZ = joystick->GetRawAxis( JOY_AXIS_RX ) * joystick->GetRawAxis( JOY_AXIS_RX ) * joystick->GetRawAxis( JOY_AXIS_RX );

        // Static starting time, updated each tick
        static double ctime = GetTime();
        // Time at current tick
        double ntime = GetTime();

        // Max allowable acceleration
        float accelFact = (ntime - ctime) * 30.334;
        // Current output values
        static float cX, cY, cZ;

        // If change greater than max allowed, only increment, else just set
        if(fabs(cX - tX) > accelFact){cX += (tX < cX)?-accelFact:accelFact;} else {cX = tX;}
        if(fabs(cY - tY) > accelFact){cY += (tY < cY)?-accelFact:accelFact;} else {cY = tY;}
        if(fabs(cZ - tZ) > accelFact){cZ += (tZ < cZ)?-accelFact:accelFact;} else {cZ = tZ;}

        // Update current time
        ctime = GetTime();

        // Actually doo dat drive thang
        if( driveRun )
            rDrive->MecanumDrive_Cartesian(cX, cY, cZ);

        Wait( .01 );
    }
}

/*********************************************************************************//**
* Input thread function
*
* Handle all of the input, as well as the ball acquisition system
*************************************************************************************/
void * inputFunc( void * arg ){

    while ( 1 ){
        // Run ball aq motors if buttons
        if( joystick->GetRawButton( JOY_BTN_LTG ) || ballAqState == 1 )
            windInBall->Set( 1.0 );
        else if( joystick->GetRawButton( JOY_BTN_LBM ) || ballAqState == -1 )
            windInBall->Set( -1.0 );
        else if( windInBall->Get( ) != 0.0 )
            windInBall->Set( 0.0 );

        // Edit reference tension value if buttons
        if( joystick->GetRawAxis( JOY_AXIS_DX ) > .5 ){
            desVoltage += 10;
            lcd->PrintfLine(DriverStationLCD::kUser_Line2, "%.0f", desVoltage );
            Wait( .1 );
        } else if( joystick->GetRawAxis( JOY_AXIS_DX ) < -.5 ){
            desVoltage -= 10;
            lcd->PrintfLine(DriverStationLCD::kUser_Line2, "%.0f", desVoltage );
            Wait( .1 );
        }

        // Raise / lower the defense pole
        if( joystick->GetRawAxis( JOY_AXIS_DY ) != poleRaiser->Get( ) )
            poleRaiser->Set( joystick->GetRawAxis( JOY_AXIS_DY ) );

        // Ball aq movement
        // Supports out, in, and neutral ( no pressure )
        if( joystick->GetRawButton( JOY_BTN_X || ballAqPistonState == 1 ) ){
            aqDeploy->Set( DoubleSolenoid::kReverse );
            aqDeploy2->Set( DoubleSolenoid::kForward );
        } else if( joystick->GetRawButton( JOY_BTN_B ) || ballAqPistonState == -1 ){
            aqDeploy2->Set( DoubleSolenoid::kReverse );
            aqDeploy->Set( DoubleSolenoid::kForward );
        } else if( joystick->GetRawButton( JOY_BTN_Y ) || ballAqPistonState == 0 ){
            aqDeploy->Set( DoubleSolenoid::kReverse );
            aqDeploy2->Set( DoubleSolenoid::kReverse );
        }

        // Just to make the field techs happy because the cRIO isnt running at 100 % CPU.
        // I really REALLY hate having to do this in a real-time system
        Wait( .01 );
    }
}

/*********************************************************************************//**
* Winch thread function
*
* Wind the winch, and fire on command
*************************************************************************************/
void * winchFunc( void * arg ){

    double pollTime = GetTime( );
                            // Used for timing the distance polling
    float maxvoltage;       // Used for smoothing out the

    while ( 1 ) {
        // Write the tension, and the limit switch state the the DS
        lcd->PrintfLine( DriverStationLCD::kUser_Line1, "%.0f", ACDC->GetVoltage( ) * 100 );
        lcd->PrintfLine( DriverStationLCD::kUser_Line3, limitSwitch->Get( ) ? "False" : "True" );

        // Write the max voltage out in the last period, on a 10 Hz rate
        //   Note: The sensor outputs continuously, we're just reading at a smaller freq
        // CANDO: Moving average on range readings
        //   NOTE: We had major issues with our sensor falling to small values unexpectedly
        //     So to remove those, we print the highest reading found during each period
        if( GetTime( ) > pollTime + .1 ){
            // Output raw voltage reading, and scaled to inches
            //   Note: The model was found experimentally with an R^2 value of .993
            lcd->PrintfLine( DriverStationLCD::kUser_Line4, "%0.3f", maxvoltage );
            lcd->PrintfLine( DriverStationLCD::kUser_Line5, "%0.1f", maxvoltage * 72.468 + 12.022 );

            // Reset max, and time
            maxvoltage = 0.0;
            pollTime = GetTime( );
        // Not ready for new period yet
        } else {
            // Update max if current reading is higher
            float voltpoll = rangeFinder->GetVoltage( );
            if( voltpoll > maxvoltage )
                maxvoltage = voltpoll;
        }

        // Turn motor on till we hit the limit switch
        // This is set up to run Async from the rest of this thread
        // Bumper starts it
        if( joystick->GetRawButton( JOY_BTN_RBM ) ){
            // If not at tension ( active low )
            if( limitSwitch->Get( ) )
                // Enable async winding
                winding = true;
            else
                // Support manual ovveride past limit switch if needed
                filthyWench->Set( 1.0 );
        // If button isn't pressed and we're winding
        } else if( winding ){
            // If we are not at limit
            if( limitSwitch->Get( ) )
                // Run the motor
                filthyWench->Set( 1.0 );
            // At limit
            else{
                // Stop async, and turn motor off
                filthyWench->Set( 0.0 );
                winding = false;
            }
        // Otherwise we're done
        } else
            // Can maybe run this only if not already at 0.0
            filthyWench->Set( 0.0 );

        // If the fire button pressed, fire and reset the catapult
        if( joystick->GetRawButton( JOY_BTN_RTG ) ){
            // Fire piston
            winchRelease->Set( DoubleSolenoid::kForward );
            Wait( 2.5 );
            winchRelease->Set( DoubleSolenoid::kReverse );
        }

        // Push all new LCD messages to the DS
        lcd->UpdateLCD( );

        // Just to make the field techs happy because the cRIO isnt running at 100 % CPU.
        // I really REALLY hate having to do this in a real-time system
        Wait( .01 );
    }
}

// Cruddy FIRST start macro function
START_ROBOT_CLASS( myRobit );
