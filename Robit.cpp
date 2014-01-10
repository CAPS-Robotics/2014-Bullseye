/*********************************************************************************//**
* @file Robit.cpp
* Source file for the myRobit class and its thread functions
*************************************************************************************/

/*************************************************************************************
* Includes
*************************************************************************************/
#include "Robit.h"

/*********************************************************************************//**
* myRobit class constructor
*
* Sets up the shooter's semaphore
*************************************************************************************/
myRobit::myRobit(){}

/*************************************************************************************
* myRobit class destructor
*************************************************************************************/
myRobit::~myRobit(){}

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
    // Set up the control objects
    compressor =            new Compressor( PRESSURE_SWITCH, COMPRESSOR_PORT );
    drivetrain =            new RobotDrive( FRONT_LEFT_MOTOR, BACK_LEFT_MOTOR, FRONT_RIGHT_MOTOR, BACK_RIGHT_MOTOR );
    joystick =              new Joystick( JOYSTICK_PORT );
    shooter_motor =         new Talon( TALON_PORT );
    shooter_piston =        new DoubleSolenoid( SHOOTER_PISTON_FORE, SHOOTER_PISTON_BACK );

    // Set all starting values for objects
    compressor->Start();
    shooter_piston->Set( shooter_piston->kReverse );

    // Start all the threads
    pthread_create( &driveThread, NULL, driveFunc, NULL );
    // pthread_create( &inputThread, NULL, inputFunc, NULL );    // Not needed for now on the kitbot.
}

/*********************************************************************************//**
* Autonomous init function
*
* Runs once at start of autonomous period. Will simply shoot four times.
*************************************************************************************/
void myRobit::AutonomousInit(){
    int                     i;              /* Counter variable                     */

    // Fire four times
    for( i = 0; i < 4; ++i ){
        sem_post( shooter_semaphore );
    }
}

/*********************************************************************************//**
* Drive thread function
*
* Drives the robot concurrently with other actuators
*************************************************************************************/
void * driveFunc( void * arg ){

    while ( 1 ){
        // Get the input values
        float tL = ( fabs( stick.GetRawAxis( 2 ) ) > .2 ) ? stick.GetRawAxis( 2 ) : 0.0;
        float tR = ( fabs( stick.GetRawAxis( 4 ) ) > .2 ) ? stick.GetRawAxis( 4 ) : 0.0;

        // Reduce power when turning
        float turnReduc = ( fabs( tL - tR ) / 8.0 );
        tL -= tL * turnReduc;
        tR -= tR * turnReduc;

        // Compensation for drivetrain un-even-ness
        tR -= tR * fabs( tL * 0.09 );

        smooth(
            &left,
            &right,
            tL,
            tR );
    }
}

/*********************************************************************************//**
* Input thread function
*
* If the shoot button is pushed, post to the shooters semaphore
*************************************************************************************/
void * inputFunc( void * arg ){

    while ( 1 ){
        // Post the semaphore if button push, debounce for 200 ms
        if( joystick->GetRawButton( BTN_X ) ){
            sem_post( shooter_semaphore );
            Wait( .2 );
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
