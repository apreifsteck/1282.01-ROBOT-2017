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

//GLOBALS
//Starting locatio: X) 5.4 Y) 26.5
/*
 Start Location:         5.699	26.400
 Drop Light Location:    11.599	15.199
 Lever Location:         12.400	46.599
 Button Location:        25.300	62.800
 */
float LIGHTX = 0;
float LIGHTY = 0;
float LEVERX = 0;
float LEVERY = 0;
float BUTTONX = 25;
float BUTTONY = 63.199;
float SATX = 0;
float SATY = 0;
float COREX = 0;
float COREY = 0;
float COREH = 0;

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
DigitalInputPin lLine(FEHIO::P0_1);
DigitalInputPin cLine(FEHIO::P0_2);
DigitalInputPin rLine(FEHIO::P0_3);

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

const float CORE_LINE_THRESHOLD;
const float DUMP_LINE_THRESHOLD;


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
void Drive(int power, float x, float y, bool bol) {
    float direction = RPS.Heading();
    float curX = RPS.X();
    float curY = RPS.Y();

    float deltaX = (x - curX);
    float deltaY = (y - curY);

    //deltaX = deltaX / (deltaX * deltaX + deltaY + deltaY);
    //deltaY = deltaY / (deltaX * deltaX + deltaY + deltaY);

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
//Goes to a certain heading?
void Rotate(int degree) {
    LCD.WriteLine(RPS.Heading());
         StopMotors();
       int power = 0;
       if(abs(RPS.Heading() - (float)degree + M_PI/4.0) < 180) {
           power = 15;
       int modifier = 1;
       if((int)(RPS.Heading() - degree + 360) % 360 < 180) {
           modifier *= 1;
       }
       else {
           power = -15;
           modifier *= -1;
       }
       int degRem = (int)(RPS.Heading() - degree + 360) % 360;
       if(degRem >= 180) {
           degRem = 360-degRem;
         }
       bool check = true;
       while(abs(RPS.Heading() - (float)degree + M_PI/4.0) > 5) {
           if(check) {
               AllMotors(power);
               Sleep(100);
           } else {
               StopMotors();
               Sleep(50);
       while(degRem > 3) {
           degRem = (int)(RPS.Heading() - degree + 360) % 360;
           if(degRem >= 180) {
               degRem = 360-degRem;
           }
           check = !check;
           int power = (degRem / 9 + 7) * modifier;
           AllMotors(power);
         }
         StopMotors();
}
       }
       }
}

bool DetectLine(AnalogInputPin sensor, float threshold) {
    if (sensor.Value() < threshold) {
        return true;
    }
   return false;
}

float DriveOnLine(int motorPercent, int moveDirection, int facingDirection, float threshold) {

   if(DetectLine(lLine, threshold) && DetectLine(rLine, threshold)) {
       Drive(motorPercent, moveDirection, facingDirection);
       } else if((DetectLine(rLine, threshold) && DetectLine(cLine, threshold)) || DetectLine(rline,threshold)) {   //to the left of the line
       Drive(10, moveDirection - 90, facingDirection);
       LCD.WriteLine("Shit is fucked.");
       } else if((DetectLine(lLine, threshold) && DetectLine(cLine, threshold)) || DetectLine(lline,threshold)) {  //to the right of the line
        Drive(10, moveDirection + 90, facingDirection);
       }
       else {
       LCD.WriteLine("Shit is suppppperrr fucked.");
       StopMotors();
   }
}

void CalibrateRPS(){
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    //-.983 flat
    //-.964
    RPS.InitializeTouchMenu();
    SD.OpenLog();

    serv.SetMin(servMin);
    serv.SetMax(servMax);

    while(switch7.Value()) {}
    LCD.WriteLine("Light Values Found.");
    LIGHTX = RPS.X();
    LIGHTY = RPS.Y();
    Sleep(1000);
    while(switch7.Value()) {}
    LCD.WriteLine("Lever Values Found.");
    LEVERX = RPS.X();
    LEVERY = RPS.Y();
    Sleep(1000);
    while(switch7.Value());
    COREX = RPS.X();
    COREY = RPS.Y();
    COREH = RPS.Heading();
    LCD.WriteLine("RPS Value, GET!!");
    while(switch6.Value()) {}
    LCD.WriteLine("Getting Ready.");
    serv.SetDegree(180);
    Sleep(3000);
}



bool StartLightOn() {
    if (cds.Value() > 1){    LCD.WriteLine(cds.Value());
    }
}

void moveToCoreLight() {
    Drive(15, NORTH, NORTH);
        while(RPS.Y() > LIGHTY + 1.25) {}
        Drive(15, WEST, NORTH);
        while(RPS.X() < LIGHTX - 0.5) {}
        StopMotors();
}

int readCoreLight() {
    if(cds.Value() < .6) {                  //RED
           LCD.WriteLine("Light: RED");
           SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
           SD.Printf("X: %f\t Y: %f", LIGHTX, LIGHTY);
           return 1; //Returns one for on red light
       } else if(cds.Value() < 1){             //BLUE
           LCD.WriteLine("Light: BLUE");
           SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
           SD.Printf("X: %f\t Y: %f", LIGHTX, LIGHTY);
           return 0; //returns 0 for on blue light
       } else
       {
           SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
           SD.Printf("X: %f\t Y: %f", LIGHTX, LIGHTY);
           LCD.WriteLine("Not on light.");
       }
       Sleep(2000);
       return -1; //for on no light
}
//Works on the assumption that you start from the switch
void PushButton() {
    Drive(20, WEST, NORTH);
    while(RPS.X() < BUTTONX);
    StopMotors();
    while(RPS.Y() < BUTTONY);
    Drive(20, SOUTH, NORTH);
}


void DumpCore(){

}

void PullLever() {

}
void TurnSatellite() {
    Drive(20,NORTH,NORTH);
    Sleep(500);
    StopMotors();
    Sleep(200);
    Drive(40,WEST, NORTH);
    while(switch3.Value() && switch2.Value());
    Drive(20,NORTH,NORTH);
    while(switch5.Value());
    Drive(20, EAST, NORTH);
    while(RPS.SatellitePercent() < 100);
    Drive(20,SOUTHEAST, NORTH);
    Sleep(500 );
    StopMotors();

}

void PushEndButton() {

}

void ResetTask(struct Task){

}

void ExtractCore(){
    int degree = COREH;
    serv.SetDegree(0);
    Sleep(500);
    Rotate(degree);
    Drive(25,COREH-90, COREH);
    Sleep(1000);
    DriveOnLine(25, COREH-90, COREH,);
    StopMotors();
    Sleep(200);
    Drive(-10, COREH-90, COREH );
    Sleep(2000);
    StopMotors();
    Sleep(200);
    for(int i = 0; i < 179; i++) {
        serv.SetDegree(i);
        Sleep(20);
    }


}

int main(int argc, const char * argv[]) {
    // insert code here...
    //CalibrateRPS();
    RPS.InitializeTouchMenu();
    float x, y;
    while(!LCD.Touch(&x, &y));
    int coreColor = -1;
    int tasks[] = {READ_LIGHT, TURN_SAT,PULL_LEVER, PUSH_BTN, EXTRACT_CORE,
                    DUMP_CORE, PUSH_END_BTN};
    for(int i = 0; i < (sizeof(tasks)/sizeof(*tasks)); i++) {
        switch (tasks[i]) {
        case READ_LIGHT:
            moveToCoreLight();
            coreColor = readCoreLight();
            break;
        case TURN_SAT:
            TurnSatellite();
            break;
        case PUSH_BTN:
            PushButton();
            break;
        case PULL_LEVER:
            PullLever();
            break;
        case EXTRACT_CORE:
            ExtractCore();
            break;
        case DUMP_CORE:
            DumpCore();
            break;
        case PUSH_END_BTN:
            PushEndButton();
            break;
        default:
            break;
        }
    }
//    while(true){
//        LCD.Write("RPS X: " );
//        LCD.WriteLine(RPS.X());
//        LCD.Write("RPS Y: " );
//        LCD.WriteLine(RPS.Y());
//        LCD.Write("RPS H: " );
//        LCD.WriteLine(RPS.Heading());

//        LCD.Clear();
//    }
    while(!LCD.Touch(&x, &y)) {
        LCD.WriteLine("Left Optosensor: %f", lLine.Value());
        LCD.WriteLine("Left Optosensor: %f", cLine.Value());
        LCD.WriteLine("Left Optosensor: %f", rLine.Value());
        Sleep(20);
        LCD.Clear();
    }

}
