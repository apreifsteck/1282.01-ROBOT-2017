//
//  main.cpp
//  Stuff
//
//  Created by Garrett Haufschild on 2/24/17.
//  Copyright © 2017 Swag Productions. All rights reserved.
//

//Included Libraries
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHAccel.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <math.h>
#include <FEHRPS.h>
#include <FEHSD.h>
#include <time.h>


/*  Average location values
 Start Location:         5.699	26.400
 Drop Light Location:    11.599	15.199
 Lever Location:         12.400	46.599
 Button Location:        25.300	62.800
 */

//GLOBALS
float LIGHTX = 11.4;
float LIGHTY = 15.4;
float LEVERX = 11.4;
float LEVERY = 46.9;
float BUTTONX = 25.9;
float BUTTONY = 62.6;
float COREX = 18.0;
float COREY = 54.3;

//MOTORS
FEHMotor fLeft(FEHMotor::Motor0, 12.0);
FEHMotor fRight(FEHMotor::Motor1, 12.0);
FEHMotor bLeft(FEHMotor::Motor2, 12.0);
FEHMotor bRight(FEHMotor::Motor3, 12.0);
FEHServo serv(FEHServo::Servo0);

//Sensors
//bump setup
//   0        1
//7              2
//
//
//
//6              3
//   5        4
DigitalInputPin switch0(FEHIO::P1_0);
DigitalInputPin switch1(FEHIO::P1_1);
DigitalInputPin switch2(FEHIO::P1_2);
DigitalInputPin switch3(FEHIO::P1_3);
DigitalInputPin switch4(FEHIO::P1_4);
DigitalInputPin switch5(FEHIO::P1_5);
DigitalInputPin switch6(FEHIO::P1_6);
DigitalInputPin switch7(FEHIO::P1_7);

//CdS Cell
AnalogInputPin cds(FEHIO::P0_0);

//Line Followers
AnalogInputPin lLine(FEHIO::P0_6);
AnalogInputPin cLine(FEHIO::P0_4);
AnalogInputPin rLine(FEHIO::P0_2);

//Constants - North is towards sea, East is lever side
const int NORTH = 0;
const int NORTHEAST = 315;
const int EAST = 270;
const int SOUTHEAST = 225;
const int SOUTH = 180;
const int SOUTHWEST = 135;
const int WEST = 90;
const int NORTHWEST = 45;
const double M_PI = 3.14159265358979323846264338327950288;
const int servMin = 990;
const int servMax = 1893;

//Movement Methods
double DegreeToRadian(int degree) {
    return (degree / 180.0) * M_PI;
    
}

void StopMotors() {
    fLeft.Stop();
    fRight.Stop();
    bLeft.Stop();
    bRight.Stop();
}

void AllMotors(int power)
{
    fLeft.SetPercent(power);
    fRight.SetPercent(power);
    bLeft.SetPercent(power);
    bRight.SetPercent(power);
}

/*
 *  Takes the direction the robot is facing and the direction it wishes to travel
 *  and uses the unit circle to find the power to apply to each motor.
 */
void Drive(int power, int moveDirection, int facingDirection) {
    StopMotors();
    int diff = moveDirection - facingDirection;
    double angle = DegreeToRadian(diff);
    angle += M_PI / 4.0;
    double xPow = cos(angle) * power;
    double yPow = sin(angle) * power;
    
    Sleep(100);
    fLeft.SetPercent(xPow);
    bRight.SetPercent(-xPow);
    fRight.SetPercent(-yPow);
    bLeft.SetPercent(yPow);
}

/*
 * Used to rotate the robot to whatever direction it needs to face,
 * rotation speed is directly proportional to the angle left to turn.
 */
void Rotate(int degree) {
    LCD.WriteLine(RPS.Heading());
    StopMotors();
    int modifier = 1;
    if((int)(RPS.Heading() - degree + 360) % 360 < 180) {
        modifier *= 1;
    }
    else {
        modifier *= -1;
    }
    int degRem = (int)(RPS.Heading() - degree + 360) % 360;
    if(degRem >= 180) {
        degRem = 360-degRem;
    }
    while(degRem > 3) {
        degRem = (int)(RPS.Heading() - degree + 360) % 360;
        if(degRem >= 180) {
            degRem = 360-degRem;
        }
        int power = (degRem / 4 + 7) * modifier;
        AllMotors(power);
    }
    StopMotors();
}

/*
 * Used to find if the robot is on the line.
 */
bool DetectLine(AnalogInputPin sensor, float threshold) {
    if (sensor.Value() < threshold) {
        return true;
    }
    return false;
}

/*
 * The robot will drive on a line found using the given threshold
 * towrads the desired direction for however long is given.
 */
float DriveOnLine(int motorPercent, int moveDirection, int facingDirection, float threshold, double elapsedTime) {
    LCD.Clear();
    double time = TimeNow();
    while(TimeNow() - time < elapsedTime)
    {
        LCD.WriteRC(lLine.Value(), 1, 1);
        LCD.WriteRC(cLine.Value(), 3, 1);
        LCD.WriteRC(rLine.Value(), 5, 1);
        if(!DetectLine(lLine, threshold) && !DetectLine(rLine, threshold) && DetectLine(cLine, threshold)) { //on the line
            fLeft.SetPercent(motorPercent);
            fRight.SetPercent(motorPercent);
            bLeft.SetPercent(-motorPercent);
            bRight.SetPercent(-motorPercent);
            LCD.WriteRC("On Line", 7 , 1);
        } else if((DetectLine(rLine, threshold) && DetectLine(cLine, threshold)) || DetectLine(rLine,threshold)) {   //to the left of the line
            fLeft.SetPercent(0);
            fRight.SetPercent(motorPercent);
            bLeft.SetPercent(-motorPercent);
            bRight.SetPercent(0);
            LCD.WriteRC("Left of line", 7 , 1);
        } else if((DetectLine(lLine, threshold) && DetectLine(cLine, threshold)) || DetectLine(lLine,threshold)) {  //to the right of the line
            fLeft.SetPercent(motorPercent);
            fRight.SetPercent(0);
            bLeft.SetPercent(0);
            bRight.SetPercent(-motorPercent);
            LCD.WriteRC("Right of line", 7 , 1);
        }
        else {  //Off line completey
            fLeft.SetPercent(0);
            fRight.SetPercent(motorPercent);
            bLeft.SetPercent(-motorPercent);
            bRight.SetPercent(0);
            LCD.WriteRC("Shit is super messed up.", 7, 1);
            StopMotors();
        }
    }
}

/*
 * Used to grab RPS values of the bin light and lever values to account
 * for different RPS values from course to course.
 */
void CalibrateRPS(){
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    
    RPS.InitializeTouchMenu();
    
    LCD.Clear( FEHLCD::Black );
    serv.SetMin(servMin);
    serv.SetMax(servMax);
    
    while(switch7.Value()) {LCD.WriteRC(RPS.X(),0,0);LCD.WriteRC(RPS.Y(),1,0);}
    LCD.WriteLine("Light Values Found.");
    LIGHTX = RPS.X();
    LIGHTY = RPS.Y();
    Sleep(1000);
    while(switch7.Value()) {LCD.WriteRC(RPS.X(),0,0);LCD.WriteRC(RPS.Y(),1,0);}
    LCD.WriteLine("Lever Values Found.");
    LEVERX = RPS.X();
    LEVERY = RPS.Y();
    Sleep(1000);
    while(switch6.Value()) {}
    LCD.WriteLine("Getting Ready.");
    serv.SetDegree(180);
    Sleep(2000);
}

/*
 * Waits until a light is detected or 30 seconds pass.
 */
bool StartLightOn() {
    double time = TimeNow();
    while(cds.Value() > 1 && TimeNow() - time < 30){}
}

/*
 * Robot moves from the start location to the core light, moves northwest
 * and then north.
 */
void moveToCoreLight() {
    Drive(30, NORTHWEST, NORTH);
    while(RPS.X() < LIGHTX - 1.5) {}
    Drive(10, EAST, NORTH);
    while(RPS.X() > LIGHTX + 0.5) {}
    Drive(30, NORTH, NORTH);
    while(RPS.Y() > LIGHTY + 1.75) {}
    Drive(10, SOUTH, NORTH);
    while(RPS.Y() < LIGHTY - 0.5) {}
    StopMotors();
}

/*
 * Readjust location until it is with half an inch of the light.
 */
void fixCoreLight() {
    Drive(10, WEST, NORTH);
    while(RPS.X() < LIGHTX - 0.5) {}
    Drive(10, EAST, NORTH);
    while(RPS.X() > LIGHTX + 0.5) {}
    Drive(10, NORTH, NORTH);
    while(RPS.Y() > LIGHTY + 0.5) {}
    Drive(10, SOUTH, NORTH);
    while(RPS.Y() < LIGHTY - 0.5) {}
    StopMotors();
}

/*
 * Attempts to read core light value, if no acceptable values are found
 * then the robot readjusts and tries again. If it still doesn't work the
 * robot assumes red and continues.
 */
int readCoreLight() {
    int value = 0;
    int i = 0;
    while(value == 0 && i < 2) {
        if(cds.Value() < .75) {                  //RED
            LCD.WriteLine("Light: RED");
            LCD.Clear( FEHLCD::Red );
            value = 1; //Returns one for on red light
        } else if(cds.Value() < 1.7){           //BLUE
            LCD.WriteLine("Light: BLUE");
            LCD.Clear( FEHLCD::Blue );
            value = -1; //returns -1 for on blue light
        } else {                                //for on no light
            LCD.WriteLine("Not on light.");
            LCD.Clear( FEHLCD::Green );
            LCD.WriteLine(cds.Value());
            fixCoreLight();
            i++;
        }
    }
    return value;
}

/*
 * Moves southwest from the lever to the button quickly. Once in line with
 * the button, drive south for 5.5 seconds to ensure button is pressed for 5
 * full seconds
 */
void PushButton() {
    Rotate(NORTH);
    Drive(100, SOUTHWEST, NORTH);
    while(RPS.X() < BUTTONX - 5.0);
    Rotate(NORTH);
    Drive(60, SOUTH, NORTH);
    while(RPS.Y() < BUTTONY - 1.0);
    Drive(10, SOUTH, NORTH);
    Sleep(5500);
}

/*
 * Moves the robot from the bottom of the ramp up and then lines up the robot
 * with the Y coordinate of the lever and then drives east until it reaches the
 * lever. It then readjusts its Y coordinate one more time before lowering the
 *  lever arm and backing up and then raising the lever arm.
 */
void PullLever() {
    Rotate(EAST);
    Drive(70, EAST, EAST);
    while(RPS.X() > 23) {}
    Drive(100, SOUTH, EAST);
    while(RPS.Y() < 40) {}
    StopMotors();
    Sleep(300);
    Drive(40, NORTH, EAST);
    while(RPS.Y() < 0 || RPS.Y() > 48.0) {
        LCD.Clear( FEHLCD::Green );
        LCD.WriteRC(RPS.X(),1,1);
        LCD.WriteRC(RPS.Y(),2,1);
    }
    StopMotors();
    Rotate(NORTH);
    Drive(40, EAST, NORTH);
    while(RPS.X() > LEVERX + 1.75) {}
    Drive(20, NORTH, NORTH);
    while(RPS.Y() > LEVERY + 0.5) {}
    Drive(20, SOUTH, NORTH);
    while(RPS.Y() < LEVERY - 0.5) {}
    serv.SetDegree(60);
    Drive(15, WEST, NORTH);
    Sleep(500);
    serv.SetDegree(180);
}

/*
 * Robot drives southwest and the west until it reaches the west wall,
 * it then drives north until it runs into the satellite and then drives
 * east until it reaches the X coordinate 25, it then drives southeast off
 * the satellite towards the bottom of the ramp.
 */
void TurnSatellite() {
    Drive(50, SOUTHWEST, NORTH);
    Sleep(200);
    Drive(100, WEST, NORTH);
    while(RPS.X() < 23.5) {}
    Drive(50, WEST, NORTH);
    while(switch6.Value() || switch7.Value()) {}
    Drive(50, NORTH,NORTH);
    while(switch1.Value()) {}
    Drive(50, EAST, NORTH);
    while(RPS.X() > 25.0) {}
    Drive(70, SOUTHEAST, NORTH);
    Sleep(375);
    StopMotors();
}

/*
 * The drives east or southeast depending on which bin it was at.
 * The robot then backs up until the stop button is press.
 */
void PushEndButton(int lightType) {
    if(lightType == 1) {
        Drive(50, SOUTHEAST, WEST);
        Sleep(300);
    }
    else {
        Drive(50, EAST, WEST);
        while(RPS.X() > 8.0){}
    }
    Drive(50, SOUTH, WEST);
}

/*
 * This method drives the robot back until it reaches the YCoord of
 * the core, then east untli the XCoord is reached, correcting itself
 * it then rotates northeast
 */
void SetupCore() {
    Drive(50, NORTH, NORTH);
    while(RPS.Y() > COREY + 2.5) {}
    Drive(7, NORTH, NORTH);
    while(RPS.Y() > COREY) {}
    Drive(7, SOUTH, NORTH);
    while(RPS.Y() < COREY) {}
    Drive(30, EAST, NORTH);
    while(RPS.X() > COREX + 2) {}
    Drive(7, EAST, NORTH);
    while(RPS.X() > COREX) {}
    Drive(7, WEST, NORTH);
    while(RPS.X() < COREX) {}
    Rotate(NORTHEAST);
}

/*
 * This method drives the robot north for .5 seconds then northeast until
 * either the X or Y coord of the core is reached, if it enters the deadzone
 * on its path it relies back the SetupCore method before or else it will
 * rotate northeast
 */
void SetupCore2() {
    Drive(20, NORTH, NORTH);
    Sleep(500);
    bool reset = false;
    Drive(50, NORTHEAST, NORTH);
    while(RPS.Y() > COREY + 2.0 && RPS.X() > COREX + 2.0) {}
    if(RPS.Y() < -1) {
        Drive(50, NORTH, NORTH);
        reset = true;
    }
    while(RPS.Y() < -1) {}
    if(reset) {
        SetupCore();
    } else {
        Rotate(NORTHEAST);
    }
}

/*
 * This method drives north for .65 seconds before rotating northeast,
 * the robot will drive east until a line is found and at the same time
 * lower the lever arm.
 */
void SetupCore3() {
    Drive(40, NORTH, NORTH);
    Sleep(650);
    Rotate(NORTHEAST);
    Drive(50, EAST, NORTHEAST);
    serv.SetDegree(0);
    while(!DetectLine(lLine, 2.0)) {LCD.WriteLine(cLine.Value());}
    
}

/*
 * Lowers the lever arm and drives southeast for a second, it then drives
 * on the line for 2 seconds, it then drives northwest for a second and then
 * it starts raising the lever arm until it is upright and keeps driving until
 * it is out of the deadzone.
 */
void ExtractCore(){
    serv.SetDegree(0);
    Drive(40, SOUTHEAST, NORTHEAST);
    Sleep(1000);
    serv.SetDegree(0);
    DriveOnLine(30, SOUTHEAST, NORTHEAST, 2.0, 2.0);
    Drive(20, NORTHWEST, NORTHEAST);
    Sleep(1000);
    for(int i = 0; i < 179; i+=3) {
        serv.SetDegree(i);
        Sleep(20);
    }
    while(RPS.X() < 0.0) {}
    StopMotors();
}

/*
 * Drives southeast on the line for 1.15 seconds and then drives backwards
 * for .5 seconds and then raises the lever arm until it is upright and
 * keeps driving until it is out of the deadzone.
 */
void ExtractCore2() {
    DriveOnLine(25, SOUTHEAST, NORTHEAST, 1.7, 1.15);
    Drive(45, NORTHWEST, NORTHEAST);
    Sleep(500);
    for(int i = 0; i < 179; i+=5) {
        serv.SetDegree(i);
        Sleep(20);
    }
    while(RPS.X() < 0.0) {}
    StopMotors();
}

/*
 * Makes the robot rotate west and drive down the hill until it is close to
 * the Y coord of the bin light, it then drives east until it is line with
 * the correct bin. It then drives forward, lowers the arm into the bin. The
 * robot then spins clockwise for .4 seconds, it then raises and lowers the lever
 * arm twice and then raises it and it rotates west again and drives south for .5
 * seconds and stops the motors.
 */
void DumpCore(int lightType) {
    Rotate(WEST);
    Drive(70, NORTH, WEST);
    double yVal = RPS.Y();
    while(yVal > LIGHTY + 5.50 || yVal < 0) {yVal = RPS.Y();}
    Rotate(WEST);
    double goalX = LIGHTX - 6.0;
    double buffer = 3.7;
    if(lightType == -1) {
        goalX = LIGHTX + 3.5;
        buffer = 0.5;
    }
    Drive(10, WEST, WEST);
    while (RPS.X() < goalX + 0.5);
    Drive(35, EAST, WEST);
    while (RPS.X() > goalX + buffer);
    Drive(10, WEST, WEST);
    while (RPS.X() < goalX);
    StopMotors();
    Rotate(WEST);
    Drive(20, NORTH, WEST);
    while(RPS.Y() > LIGHTY + 0.8) {}
    StopMotors();
    serv.SetDegree(0);
    Sleep(300);
    AllMotors(30);
    Sleep(400);
    StopMotors();
    serv.SetDegree(180);
    Sleep(300);
    serv.SetDegree(0);
    Sleep(300);
    serv.SetDegree(180);
    Sleep(300);
    serv.SetDegree(0);
    Sleep(300);
    serv.SetDegree(180);
    Rotate(WEST);
    Drive(50, SOUTH, WEST);
    Sleep(500);
    StopMotors();
}

/*      Used as testing method for getting needed values of a new environment
 int main(void) {
 RPS.InitializeTouchMenu();
 SD.OpenLog();
 LCD.Clear();
 while(true) {
 LCD.WriteRC(RPS.X(),1,0);
 LCD.WriteRC(RPS.Y(),2,0);
 LCD.WriteRC(lLine.Value(),3,0);
 LCD.WriteRC(cLine.Value(),4,0);
 LCD.WriteRC(rLine.Value(),5,0);
 LCD.WriteAt(cds.Value(),6,0);
 if(!switch7.Value()) {
 SD.Printf("X: %f\n", RPS.X());
 SD.Printf("Y: %f\n", RPS.Y());
 SD.Printf("Left: %f\n", lLine.Value());
 SD.Printf("Center: %f\n", cLine.Value());
 SD.Printf("Right: %f\n", rLine.Value());
 SD.Printf("CDS: %f\n\n", cds.Value());
 Sleep(500);
 }
 }
 SD.CloseLog();
 }
 */

/*
 * Main method the completes all the tasks in the correct order.
 */
int main(void) {
    CalibrateRPS();
    int lightType = 1;
    StartLightOn();
    moveToCoreLight();
    lightType = readCoreLight();
    if(lightType==0) {
        lightType = 1;
    }
    TurnSatellite();
    PullLever();
    PushButton();
    SetupCore3();
    ExtractCore2();
    DumpCore(lightType);
    PushEndButton(lightType);
}
