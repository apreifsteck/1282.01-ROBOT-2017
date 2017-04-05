//
//  main.cpp
//  Stuff
//
//  Created by Garrett Haufschild on 2/24/17.
//  Copyright © 2017 Swag Productions. All rights reserved.
//

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


//GLOBALS
//Starting location: X) 5.4 Y) 26.5
/*
 Start Location:         5.699	26.400
 Drop Light Location:    11.599	15.199
 Lever Location:         12.400	46.599
 Button Location:        25.300	62.800
 */
float LIGHTX = 11.4;
float LIGHTY = 15.4;
float LEVERX = 11.4;
float LEVERY = 46.9;
float BUTTONX = 25;
float BUTTONY = 63.199;
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

const float CORE_LINE_THRESHOLD = 0;
const float DUMP_LINE_THRESHOLD = 0;


//tasks
#define TURN_SAT 0
#define READ_LIGHT 1
#define PUSH_BTN 2
#define PULL_LEVER 3
#define EXTRACT_CORE 4
#define DUMP_CORE 5
#define PUSH_END_BTN 6

#define RED_LIGHT 1

//Movement Methods




struct Task{
    float startX;
    float startY;
    int direction;
    bool completed;
    float timeout;
};

struct State{
    float currentX;
    float currentY;
    int currentDirection;
    float time;
};



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
 * USE WHEN RPS IS PRESENT
 * Uses current information to find where to move
 */
void Drive(int power, float x, float y) {
    float direction = RPS.Heading();
    while((int)direction % 45 > 2) {
        AllMotors(8);
    }
    
    float curX = RPS.X();
    float curY = RPS.Y();
    
    float deltaX = (x - curX);
    float deltaY = (y - curY);
    
    float arctan = (deltaY/deltaX) + 180 + M_PI/4.0;
    float degree = direction - arctan;
    
    
    deltaX = cos(degree);
    deltaY = sin(degree);
    
    double xPow = deltaX * power;
    double yPow = deltaY * power;
    
    StopMotors();
    
    fLeft.SetPercent(xPow);
    bRight.SetPercent(-xPow);
    fRight.SetPercent(-yPow);
    bLeft.SetPercent(yPow);
    LCD.Clear();
    while(abs(RPS.X() - x) > 1 || abs(RPS.Y() - y) > 1) {
        SD.Printf("RPS Heading: \t%f\t%f\n", RPS.Heading(), degree);
        SD.Printf("X: %f     Y: %f\n", RPS.X(), RPS.Y());
        SD.Printf("Goal X: %f     Goal Y: %f\n\n", x, y);
        
        LCD.WriteRC(RPS.X(),1,1);
        LCD.WriteRC(RPS.Y(),1,7);
        LCD.WriteRC(abs(RPS.X() - x),3,1);
        LCD.WriteRC(abs(RPS.Y() - y),3,7);
    }
    StopMotors();
}

/*
 *  FOR USE WHEN NO RPS IS PRESENT
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

void RotateSlow(int degree) {
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
        int power = (degRem / 7 + 7) * modifier;
        AllMotors(power);
    }
    StopMotors();
}

bool DetectLine(AnalogInputPin sensor, float threshold) {
    if (sensor.Value() < threshold) {
        return true;
    }
    return false;
}

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
            //Drive(10, moveDirection - 45, facingDirection);
            fLeft.SetPercent(0);
            fRight.SetPercent(motorPercent);
            bLeft.SetPercent(-motorPercent);
            bRight.SetPercent(0);
            LCD.WriteRC("Left of line", 7 , 1);
        } else if((DetectLine(lLine, threshold) && DetectLine(cLine, threshold)) || DetectLine(lLine,threshold)) {  //to the right of the line
            //Drive(10, moveDirection + 45, facingDirection);
            fLeft.SetPercent(motorPercent);
            fRight.SetPercent(0);
            bLeft.SetPercent(0);
            bRight.SetPercent(-motorPercent);
            LCD.WriteRC("Right of line", 7 , 1);
        }
        else {
            fLeft.SetPercent(0);
            fRight.SetPercent(motorPercent);
            bLeft.SetPercent(-motorPercent);
            bRight.SetPercent(0);
            LCD.WriteRC("Shit is suppppperrr fucked.", 7, 1);
            StopMotors();
        }
    }
}

void CalibrateRPS(){
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    
    RPS.InitializeTouchMenu();
    SD.OpenLog();
    
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
    //    while(switch7.Value()) {LCD.WriteRC(RPS.X(),0,0);LCD.WriteRC(RPS.Y(),1,0);}
    //    COREX = RPS.X();
    //    COREY = RPS.Y();
    //    LCD.WriteLine("Core Values Found.");
    while(switch6.Value()) {}
    LCD.WriteLine("Getting Ready.");
    serv.SetDegree(180);
    Sleep(2000);
}



bool StartLightOn() {
    while(cds.Value() > 1){    LCD.WriteLine(cds.Value());  }
}

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

int readCoreLight() {
    int value = 0;
    while(value == 0) {
        if(cds.Value() < .55) {                  //RED
            LCD.WriteLine("Light: RED");
            LCD.Clear( FEHLCD::Red );
            value = 1; //Returns one for on red light
        } else if(cds.Value() < 1){             //BLUE
            LCD.WriteLine("Light: BLUE");
            LCD.Clear( FEHLCD::Blue );
            value = -1; //returns 0 for on blue light
        } else {
            LCD.WriteLine("Not on light.");
            LCD.Clear( FEHLCD::Green );
            fixCoreLight();
        }
    }
    return value; //for on no light
}

//Works on the assumption that you start from the switch
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

void PullLever() {
    Rotate(EAST);
    Drive(70, EAST, EAST);
    while(RPS.X() > 23) {}
    Drive(100, SOUTH, EAST);
    while(RPS.Y() < 40) {}
    StopMotors();
    Sleep(300);
    Drive(40, NORTH, EAST);
    while(RPS.Y() < 0 || RPS.Y() > 47.5) {
        LCD.Clear( FEHLCD::Green );
        LCD.WriteRC(RPS.X(),1,1);
        LCD.WriteRC(RPS.Y(),2,1);
    }
    SD.Printf("%f, %f\n", RPS.X(), RPS.Y());
    StopMotors();
    Rotate(NORTH);
    SD.Printf("%f, %f\n", RPS.X(), RPS.Y());
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

void PushEndButton(int lightType) {
    SD.CloseLog();
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

void ResetTask(struct Task){
    
}

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

void SetupCore3() {
    Drive(40, NORTH, NORTH);
    Sleep(650);
    Rotate(NORTHEAST);
    Drive(50, EAST, NORTHEAST);
    serv.SetDegree(0);
    while(!DetectLine(lLine, 2.0)) {LCD.WriteLine(cLine.Value());}
    
}

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

void ExtractCore2() {
    DriveOnLine(25, SOUTHEAST, NORTHEAST, 1.7, 1.5);
    Drive(45, NORTHWEST, NORTHEAST);
    Sleep(500);
    for(int i = 0; i < 179; i+=5) {
        serv.SetDegree(i);
        Sleep(20);
    }
    while(RPS.X() < 0.0) {}
    StopMotors();
}

void DumpCore(int lightType) {
    Rotate(WEST);
    Drive(70, NORTH, WEST);
    while(RPS.Y() > LIGHTY + 5.50 || RPS.Y() < 0);
    Rotate(WEST);
    double goalX = 5;
    double buffer = 4.2;
    if(lightType == -1) {
        goalX = 15.0;
        buffer = 1.0;
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
    serv.SetDegree(90);
    Sleep(150);
    serv.SetDegree(0);
    Sleep(150);
    serv.SetDegree(90);
    Sleep(150);
    serv.SetDegree(0);
    Sleep(150);
    serv.SetDegree(90);
    Sleep(150);
    serv.SetDegree(0);
    Sleep(150);
    serv.SetDegree(180);
    Drive(50, SOUTH, WEST);
    Sleep(500);
    StopMotors();
}

//int main(void) {
//    RPS.InitializeTouchMenu();
//    LCD.Clear();
//    while(true) {
//    LCD.WriteRC(RPS.X(),0,0);LCD.WriteRC(RPS.Y(),1,0);
//    }
//    //LIGHT: 11.4, 15.4
//    //BINRED: 6.8, 14.6
//    //BINBLUE: 14.9 14.8
//    //WALL: 31.5
//    //RAMP: 18.9
//    //LEVER: 47.2, 12.0
//    //CORE: 18.9, 54.7
//    //END: 6.0
//}


int main(void) {
    CalibrateRPS();
    int lightType = 1;
    StartLightOn();
    moveToCoreLight();
    lightType = readCoreLight();
    TurnSatellite();
    PullLever();
    PushButton();
    SetupCore3();
    ExtractCore2();
    DumpCore(lightType);
    PushEndButton(lightType);
    LCD.WriteLine("Finished");
    
    //24 away from core
    //72 is max y, 36 is max x
    //put it 18 away from maxes
}


/*
 int main(void) {
 while(true) {
 LCD.WriteRC(lLine.Value(),0,0);
 LCD.WriteRC(cLine.Value(),1,0);
 LCD.WriteRC(rLine.Value(),2,0);
 }
 }
 */
