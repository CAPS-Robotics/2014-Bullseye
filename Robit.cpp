/*********************************************************************************//**
* @file Robit.cpp
* Source file for the myRobit class and its thread functions
*************************************************************************************/


/*************************************************************************************
* TODO:
* 
* Adjustable desVoltage
* Winch release
*************************************************************************************/


/*************************************************************************************
* Includes
*************************************************************************************/
#include "Robit.h"

/*************************************************************************************
* Global variable definitions
*************************************************************************************/
float                desVoltage = 2.5;      /**< Desired voltage on load cell       */

/*********************************************************************************//**
* myRobit class constructor
*
* Sets up the shooter's semaphore
*************************************************************************************/
myRobit::myRobit(){};

/*************************************************************************************
* myRobit class destructor
*************************************************************************************/
myRobit::~myRobit(){};

/*************************************************************************************
Unused class functions
*************************************************************************************/
void myRobit::DisabledInit(){};
void myRobit::TeleopInit(){};
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
    // Set up the drive talons
    lDriveTalon =           new Talon( L_DRIVE_PORT );
    rDriveTalon =           new Talon( R_DRIVE_PORT );

    // Set up the control objects
    ACDC =                  new AnalogChannel( LOADCELL_CHANNEL );
    filthyWench =           new Talon( WINCH_PORT );
    joystick =              new Joystick( JOYSTICK_PORT );

    // Set all starting values for objects
    compressor->Start();
    shooter_piston->Set( shooter_piston->kReverse );

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
    // Nada por ahora
}

/*********************************************************************************//**
* Drive thread function
*
* Drives the robot concurrently with other actuators
*************************************************************************************/
void * driveFunc( void * arg ){

    while ( 1 ){
        // Get the input values
        float tL = ( fabs( stick.GetRawAxis( 2 ) ) > DEAD_ZONE ) ? stick.GetRawAxis( 2 ) : 0.0;
        float tR = ( fabs( stick.GetRawAxis( 4 ) ) > DEAD_ZONE ) ? stick.GetRawAxis( 4 ) : 0.0;

        // Reduce power when turning
        float turnReduc = ( fabs( tL - tR ) / TURN_REDUC_FACTOR );
        tL -= tL * turnReduc;
        tR -= tR * turnReduc;

        // Compensation for drivetrain un-even-ness
        tR -= tR * fabs( tL * UNEVEN_ADJ_FACTOR );

        smooth(
            lDriveTalon,
            rDriveTalon,
            tL,
            tR );
    }
}

/*********************************************************************************//**
* Input thread function
*
* N/A for now
*************************************************************************************/
void * inputFunc( void * arg ){

    while ( 1 ){
        // Do nothing for now
    }
}

/*********************************************************************************//**
* Winch thread function
*
* If button is pushed, wind the winch
*************************************************************************************/
void * winchFunc( void * arg ){

    while ( 1 ) {
        if ( joystick->GetRawButton( BTN_LBM ) ) {
            // Turn motor on till we are at tension
            filthyWench->Set( 1.0 );
            while ( ACDC->GetVoltage() < desVoltage );
            filthyWench->Set( 0.0 );
        }
    }
}

/*********************************************************************************//**
* Smooth helper function
*
* Takes the outputs to the motors, and smooths them up to the desired value, based
* on time taken since the last loop
*************************************************************************************/
void smooth( Talon * lTalon, Talon * rTalon, float lVal, float rVal ){
    static double prevTime = GetTime( );    /* Persistant previous time             */
    static float currL, currR;              /* Persistant current motor speeds      */

    // Get the time at this loop
    double nowTime = GetTime( );

    // Calculate maximum acceleration for this tick
    // PS -> WTF does that magic number stand for? No idea!!
    //   Make it smaller, it accelerates really slowly
    //   Any larger, and smoothing is not doing much at all
    float accelMax = ( nowTime - prevTime ) * 38.334;

    // If we want to accelerate faster than allowed, don't
    // Also handle the + and - cases
    // Because failure to do so means it never stops accelerating.....
    //   I may have dented the wall figuring that out...
    currL = ( fabs( lVal - currL ) > accelMax ) ? ( ( lVal < currL ) ? currL - accelMax : currL + accelMax ) : lVal;
    currR = ( fabs( rVal - currR ) > accelMax ) ? ( ( rVal < currR ) ? currR - accelMax : currR + accelMax ) : rVal;

    // Now set the motors for tank drive
    lTalon->Set( currL );
    rTalon->Set( currX );
}
// Cruddy FIRST start macro function
START_ROBOT_CLASS( myRobit );
