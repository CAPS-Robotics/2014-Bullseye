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
myRobit::myRobit(){
    // Set up the shooter's semaphore
    shooter_semaphore = sem_open( "SHOOTER", O_CREAT, 0644, 0 );
}

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
    pthread_create( &inputThread, NULL, inputFunc, NULL );
    pthread_create( &shooterThread, NULL, shooterFunc, NULL );
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
        // Do dat drive thang
        drivetrain->MecanumDrive_Cartesian(
            joystick->GetX( ),
            joystick->GetY( ),
            joystick->GetZ( ) );
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
* Shooter thread function
*
* Waits on the shooter's semaphore, then fires
*************************************************************************************/
void * shooterFunc( void * arg ){
    int                     val;            /* Used for checking semaphore value    */
    int                     sem_retval;     /* Used for checking semaphore wait     */

    while( 1 ){
        // Try to wait for the semaphore
        sem_retval = sem_trywait( shooter_semaphore );

        // If we got the semaphore lock
        if( sem_retval == 0 ){
            // Start the shooter motor
            shooter_motor->Set( 1.0 );
            Wait( 2.75 );
    
            // Pulse the firing piston
            shooter_piston->Set( shooter_piston->kForward );
            Wait( .75 );
            shooter_piston->Set( shooter_piston->kReverse );
    
            // Only stop the motor if we aren't firing again
            sem_getvalue( shooter_semaphore, &val );
            if( !val ){
                shooter_motor->Set( 0.0 );
            }
            
        } else {
            // If manual mode button is held
            if( joystick->GetRawButton( BTN_LBM ) ){
                // If the motor trigger held
                if( joystick->GetRawButton( BTN_LTG ) ){
                    // Turn motor on
                    shooter_motor->Set( 1.0 );
                } else {
                    // Keep motor off
                    shooter_motor->Set( 0.0 );
                }
                
                // If the piston trigger held
                if(joystick->GetRawButton( BTN_RTG ) ){
                    // Extend the piston
                    shooter_piston->Set( shooter_piston->kForward );
                } else {
                    // Retract the piston
                    shooter_piston->Set( shooter_piston->kReverse );
                }
            }
        }
    }
}

// Cruddy FIRST start macro function
START_ROBOT_CLASS( myRobit );
