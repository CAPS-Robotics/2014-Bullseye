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
float                desVoltage = 290;      /**< Desired voltage on load cell       */

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
    // Nada ahora
}

/*********************************************************************************//**
* Teleop init function
*
* Runs once at start of teleop period.
*************************************************************************************/
void myRobit::TeleopInit(){
    // Start the compressor
    compressor->Start( );
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
        // Run motors if button
        if( joystick->GetRawButton( JOY_BTN_LTG ) )
            windInBall->Set( 1.0 );
        else if( joystick->GetRawButton( JOY_BTN_LBM ) )
            windInBall->Set( -1.0 );
        else if( windInBall->Get( ) != 0.0 )
            windInBall->Set( 0.0 );

        // Edit desvoltage if buttons
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

        //
        if( joystick->GetRawButton( JOY_BTN_X ) ){
            aqDeploy->Set( DoubleSolenoid::kReverse );
            aqDeploy2->Set( DoubleSolenoid::kForward );
        } else if( joystick->GetRawButton( JOY_BTN_B ) ){
            aqDeploy2->Set( DoubleSolenoid::kReverse );
            aqDeploy->Set( DoubleSolenoid::kForward );
        } else if( joystick->GetRawButton( JOY_BTN_Y ) ){
            aqDeploy->Set( DoubleSolenoid::kReverse );
            aqDeploy2->Set( DoubleSolenoid::kReverse );
        }

        Wait( .01 );
    }
}

/*********************************************************************************//**
* Winch thread function
*
* Wind the winch, and fire on command
*************************************************************************************/
void * winchFunc( void * arg ){

    bool winding;

    while ( 1 ) {
        lcd->PrintfLine( DriverStationLCD::kUser_Line1, "%.0f", ACDC->GetVoltage( ) * 100 );
        lcd->PrintfLine( DriverStationLCD::kUser_Line3, limitSwitch->Get( ) ? "False" : "True" );
        lcd->PrintfLine( DriverStationLCD::kUser_Line4, "%0.1f", rangeFinder->GetVoltage( )* 275.0 - .75);

        // Turn motor on till we hit the limit switch
        if( joystick->GetRawButton( JOY_BTN_RBM ) ){
            if( limitSwitch->Get( ) )
                winding = true;
            else
                filthyWench->Set( 1.0 );
        } else if( winding ){
            if( limitSwitch->Get( ) )
                filthyWench->Set( 1.0 );
            else{
                filthyWench->Set( 0.0 );
                winding = false;
            }
        } else
            filthyWench->Set( 0.0 );

        if( joystick->GetRawButton( JOY_BTN_RTG ) ){
            // Fire piston
            winchRelease->Set( DoubleSolenoid::kForward );
            Wait( 2.5 );
            winchRelease->Set( DoubleSolenoid::kReverse );
        }

        lcd->UpdateLCD( );

        Wait( .01 );
    }
}

// Cruddy FIRST start macro function
START_ROBOT_CLASS( myRobit );
