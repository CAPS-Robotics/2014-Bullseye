/*********************************************************************************//**
* @file Robit.cpp
* Source file for the myRobit class and its thread functions
*************************************************************************************/

/*************************************************************************************
* TODO:
*
* Adjustable desVoltage
*************************************************************************************/

/*************************************************************************************
* Includes
*************************************************************************************/
#include "Robit.h"

/*************************************************************************************
* Global variable definitions
*************************************************************************************/
float                desVoltage = 1.2;      /**< Desired voltage on load cell       */

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
    // Set up the drive object
    rDrive =                new RobotDrive( LF_DRIVE_PORT, LR_DRIVE_PORT, RF_DRIVE_PORT, RR_DRIVE_PORT );

    // Set up the control objects
    ACDC =                  new AnalogChannel( LOADCELL_CHANNEL );
    aqDeploy =              new DoubleSolenoid( BALLAQ_DEPLOY, BALLAQ_UNDEPLOY );
    compressor =            new Compressor( PRESSURE_SWITCH, COMPRESSOR_PORT );
    filthyWench =           new Talon( WINCH_PORT );
    joystick =              new Joystick( JOY_PORT_1 );
    winchRelease =          new Solenoid( WINCH_RELEASE_PORT );
    windInBall =            new Talon( BALLAQ_WINDIN );

    // Set all starting values for objects
    aqDeploy->Set( aqDeploy::kReverse );
    compressor->Start( );
    winchRelease->Set( false );

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
        rDrive->MecanumDrive_Cartesian(cX, cY, cZ);
    }
}

/*********************************************************************************//**
* Input thread function
*
* Handle all of the input, as well as the ball acquisition system
*************************************************************************************/
void * inputFunc( void * arg ){

    while ( 1 ){
        // Deploy / Undeploy on buttons
        // Run motors if deployed, and button
        if( joystick->GetRawButton( JOY_BTN_RBM ) )
            aqDeploy->Set( aqDeploy::kForward );
        else if( joystick->GetRawButton( JOY_BTN_LBM ) )
            aqDeploy->Set( aqDeploy::kReverse );

        if( joystick->GetRawButton( JOY_BTN_LTG ) )
            windInBall->Set( 1.0 );
        else
            windInBall->Set( 0.0 );

        // Edit desvoltage if buttons
        if( joystick->GetRawAxis( JOY_AXIS_DY ) > .5 ){
            desVoltage += .1;
            Wait( .1 );
        } else if( joystick->GetRawAxis( JOY_AXIS_DY ) < -.5 ){
            desVoltage -= .1;
            Wait( .1 );
        }
    }
}

/*********************************************************************************//**
* Winch thread function
*
* Wind the winch, and fire on command
*
* TODO: Do this with a semaphore... waitlock instead of the while, and move that to input thread
*************************************************************************************/
void * winchFunc( void * arg ){

    while ( 1 ) {

        // Turn motor on till we are at tension
        filthyWench->Set( 1.0 );
        while ( ACDC->GetVoltage() < desVoltage );
        filthyWench->Set( 0.0 );

        while ( ! joystick->GetRawButton( JOY_BTN_RTG ) );

        // Fire piston
        winchRelease->Set( true );
        Wait( 2.5 );
        winchRelease->Set( false );
    }
}

// Cruddy FIRST start macro function
START_ROBOT_CLASS( myRobit );
