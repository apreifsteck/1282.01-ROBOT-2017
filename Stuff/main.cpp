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
AnalogInputPin lLine(FEHIO::P0_2);
AnalogInputPin cLine(FEHIO::P0_4);
AnalogInputPin rLine(FEHIO::P0_6);

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
        int power = (degRem / 9 + 7) * modifier;
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

float DriveOnLine(int motorPercent, int moveDirection, int facingDirection, float threshold) {
    int time = RPS.Time();
    while(time - RPS.Time() > 3)
    {
        if(DetectLine(lLine, threshold) && DetectLine(rLine, threshold)) {
            Drive(motorPercent, moveDirection, facingDirection);
        } else if((DetectLine(rLine, threshold) && DetectLine(cLine, threshold)) || DetectLine(rLine,threshold)) {   //to the left of the line
            Drive(10, moveDirection - 90, facingDirection);
            LCD.WriteLine("Shit is fucked.");
        } else if((DetectLine(lLine, threshold) && DetectLine(cLine, threshold)) || DetectLine(lLine,threshold)) {  //to the right of the line
            Drive(10, moveDirection + 90, facingDirection);
        }
        else {
            LCD.WriteLine("Shit is suppppperrr fucked.");
            StopMotors();
        }
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
    while(switch7.Value()) {LCD.WriteRC(RPS.X(),0,0);LCD.WriteRC(RPS.Y(),1,0);}
    COREX = RPS.X();
    COREY = RPS.Y();
    COREH = RPS.Heading();
    LCD.WriteLine("Core Values Found.");
    while(switch6.Value()) {}
    LCD.WriteLine("Getting Ready.");
    serv.SetDegree(180);
    Sleep(3000);
}



bool StartLightOn() {
    while(cds.Value() > 1){    LCD.WriteLine(cds.Value());  }
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
        return -1; //returns 0 for on blue light
    } else
    {
        SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
        SD.Printf("X: %f\t Y: %f", LIGHTX, LIGHTY);
        LCD.WriteLine("Not on light.");
    }
    Sleep(2000);
    return 0; //for on no light
}

//Works on the assumption that you start from the switch
void PushButton() {
    Drive(20, WEST, NORTH);
    while(RPS.X() < BUTTONX - 1);
    Rotate(NORTH);
    Drive(20, SOUTH, NORTH);
    while(RPS.Y() < BUTTONY - 2);
    Drive(10, SOUTH, NORTH);
    Sleep(7000);
}

void PullLever() {
    
}

void TurnSatellite() {
    Drive(20,SOUTH, NORTH);
    Sleep(1000);
    Drive(40,WEST, NORTH);
    while(switch6.Value() || switch7.Value()) {}
    Drive(20,NORTH,NORTH);
    while(switch1.Value()) {}
    Drive(20, EAST, NORTH);
    while(RPS.SatellitePercent() < 90) {LCD.WriteLine(RPS.SatellitePercent());}
    Drive(20, SOUTH, NORTH);
    Sleep(500);
    StopMotors();
}

void PushEndButton() {
    
}

void ResetTask(struct Task){
    
}

void ExtractCore(){
    serv.SetDegree(0);
    Sleep(500);
    Drive(25, SOUTHEAST, NORTHEAST);
    Sleep(2500);
    Drive(10, SOUTHEAST, NORTHEAST);
    Sleep(1000);
    //DriveOnLine(15, SOUTHEAST, NORTHEAST, 2.7);
    //StopMotors();
    Drive(20, NORTHWEST, NORTHEAST);
    Sleep(3000);
    StopMotors();
    Sleep(200);
    for(int i = 0; i < 179; i++) {
        serv.SetDegree(i);
        Sleep(20);
    }
}

void DumpCore(int lightType) {
    Drive(30,NORTH,EAST);
    while(RPS.Y() > LIGHTY + 4.50);
    Drive(20, EAST, EAST);
    while (RPS.X() > LIGHTX + 1.0);
    StopMotors();
    Rotate(WEST);
    int pow = lightType * 15;
    Drive(pow, EAST, WEST);
    Sleep(1500);
    Drive(20, NORTH, WEST);
    Sleep(1000);
    StopMotors();
    serv.SetDegree(0);
    Sleep(100);
    serv.SetDegree(180);
    Sleep(100);
    serv.SetDegree(0);
    Sleep(100);
    serv.SetDegree(180);
    Drive(30, SOUTH, WEST);
    Sleep(1000);
    StopMotors();
}

/*
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
 */

//void testLine()
//{
//    DriveOnLine(20);
//}

int main(void) {
    CalibrateRPS();
    StartLightOn();
    moveToCoreLight();
    int lightType = readCoreLight();
    TurnSatellite();
    Rotate(EAST);
    Drive(20, EAST, EAST);
    while(RPS.X() > 19.5) {}
    Drive(100, SOUTH, EAST);
    while(RPS.Y() < 40) {}
    StopMotors();
    Sleep(500);
    Rotate(NORTH);
    Drive(10, NORTH, NORTH);
    while(RPS.Y() > LEVERY + 0.5) {}
    Drive(20, EAST, NORTH);
    while(RPS.X() > LEVERX + 1) {}
    serv.SetDegree(60);
    Drive(15, WEST, NORTH);
    Sleep(1500);
    serv.SetDegree(180);
    PushButton();
    Drive(20, NORTH, NORTH);
    while(RPS.Y() > COREY + 2) {}
    Drive(7, NORTH, NORTH);
    while(RPS.Y() > COREY) {}
    Drive(20, WEST, NORTH);
    while(RPS.X() < COREX-2) {}
    Drive(7, WEST, NORTH);
    while(RPS.X() < COREX) {}
    Drive(7, EAST, NORTH);
    while(RPS.X() > COREX) {}
    Rotate(NORTHEAST);
    ExtractCore();
    Rotate(EAST);
    DumpCore(lightType);
    Drive(10, EAST, WEST);
    while(switch5.Value() || switch4.Value()){}
    Drive(15, WEST, WEST);
    Sleep(500);
    Drive(20, SOUTH, WEST);
    LCD.WriteLine("Finished");
    
    //24 away from core
    //72 is max y, 36 is max x
    //put it 18 away from maxes
}

/*
 int main(void) {
 LCD.Clear( FEHLCD::Black );
 LCD.SetFontColor( FEHLCD::White );
 
 //-.983 flat
 //-.964
 RPS.InitializeTouchMenu();
 SD.OpenLog();
 
 serv.SetMin(servMin);
 serv.SetMax(servMax);
 float LIGHTX = 0;
 float LIGHTY = 0;
 float LEVERX = 0;
 float LEVERY = 0;
 
 SD.Printf("1st: %f\t%f", RPS.X(), RPS.Y());
 Drive(20, 15, NORTH);
 Sleep(2000);
 SD.Printf("2nd: %f\t%f", RPS.X(), RPS.Y());
 Drive(20, 165, NORTH);
 Sleep(2000);
 SD.Printf("3rd: %f\t%f", RPS.X(), RPS.Y());
 Drive(20, 285, NORTH);
 Sleep(2000);
 SD.Printf("4th: %f\t%f", RPS.X(), RPS.Y());
 SD.CloseLog();
 }
 */

/*
 int main(void) {
 LCD.Clear( FEHLCD::Black );
 LCD.SetFontColor( FEHLCD::White );
 
 //-.983 flat
 //-.964
 RPS.InitializeTouchMenu();
 SD.OpenLog();
 
 serv.SetMin(servMin);
 serv.SetMax(servMax);
 float LIGHTX = 0;
 float LIGHTY = 0;
 float LEVERX = 0;
 float LEVERY = 0;
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
 while(switch6.Value()) {}
 LCD.WriteLine("Getting Ready.");
 serv.SetDegree(180);
 Sleep(3000);
 
 while(cds.Value() > 1){    LCD.WriteLine(cds.Value());
 }
 
 Drive(15, NORTH, NORTH);
 while(RPS.Y() > LIGHTY + 1.25) {}
 Drive(15, WEST, NORTH);
 while(RPS.X() < LIGHTX - 0.5) {}
 StopMotors();
 if(cds.Value() < .6) {                  //RED
 LCD.WriteLine("Light: RED");
 SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
 SD.Printf("X: %f\t Y: %f", LIGHTX, LIGHTY);
 } else if(cds.Value() < 1){             //BLUE
 LCD.WriteLine("Light: BLUE");
 SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
 SD.Printf("X: %f\t Y: %f", LIGHTX, LIGHTY);
 } else
 {
 SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
 SD.Printf("X: %f\t Y: %f", LIGHTX, LIGHTY);
 LCD.WriteLine("Not on light.");
 }
 Sleep(2000);
 
 //LCD.WriteLine("Move west until switch are pressed");
 
 Drive(40, WEST, NORTH);
 while(RPS.X() < 14.5) {}
 
 StopMotors();
 
 //LCD.WriteLine("Drive up hill.");
 Drive(100, SOUTH, NORTH);
 //    while(Accel.Z() < -.970) {}
 //    LCD.WriteLine("On Hill.");
 //    while(Accel.Z() > -.965) {}
 //    LCD.WriteLine("Off Hill.");
 
 while(RPS.Y() < 45) {}
 StopMotors();
 Sleep(500);
 if(!(((int)RPS.Heading() + 10) % 360 < 20))
 Rotate(NORTH);
 Drive(20, NORTH, NORTH);
 
 while(RPS.Y() > LEVERY + 1.25) {}
 
 Drive(20, EAST, NORTH);
 
 while(RPS.X() > LEVERX + 0.5) {}
 SD.Printf("X: %f\t Y: %f", RPS.X(), RPS.Y());
 SD.Printf("X: %f\t Y: %f", LEVERX, LEVERY);
 
 StopMotors();
 serv.SetDegree(60);
 Drive(15, WEST, NORTH);
 Sleep(1000);
 serv.SetDegree(180);
 StopMotors();
 LCD.WriteLine("End Program.");
 SD.CloseLog();
 
 //RPS.InitializeTouchMenu();
 //SD.OpenLog();
 //while(true){ Sleep(100); LCD.WriteLine(cds.Value());}
 //Drive(30, 5.7, 26.4, true);
 //SD.CloseLog();
 }
 */
/*
 int main(void) {
 
 // insert code here...
 int tasks[] = {TURN_SAT, PUSH_BTN, PULL_LEVER,
 EXTRACT_CORE, DUMP_CORE, PUSH_END_BTN};
 for(int i = 0; i < tasks.length(); i++) {
 switch (tasks[i]) {
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
 
 //    fLeft.SetPercent(15);
 //    Sleep(2000);
 //    StopMotors();
 //    fRight.SetPercent(15);
 //    Sleep(2000);
 //    StopMotors();
 //    bLeft.SetPercent(15);
 //    Sleep(2000);
 //    StopMotors();
 //    bRight.SetPercent(15);
 //    Sleep(2000);
 //    StopMotors();
 
 LCD.Clear( FEHLCD::Black );
 LCD.SetFontColor( FEHLCD::White );
 
 //-.983 flat
 //-.964
 
 LCD.WriteLine("Waiting for light.");
 while(cds.Value() > 1){    LCD.WriteLine(cds.Value());
 }
 
 LCD.WriteLine("Light has been found.");
 
 //Drive forward for 1 second
 Drive(25, NORTH, NORTH);
 Sleep(1250);
 
 LCD.WriteLine("Move west until switch are pressed");
 
 Drive(40, WEST, NORTH);
 while(switch7.Value() || switch6.Value()){}
 StopMotors();
 
 
 //ASSURE YOU ARE AGAINST WALL
 if(switch7.Value()) //7 is unpressed
 {
 bLeft.SetPercent(50);
 bRight.SetPercent(50);
 while(switch7.Value()) {}
 }
 else if(switch6.Value())
 {
 fLeft.SetPercent(50);
 fRight.SetPercent(50);
 while(switch6.Value()) {}
 }
 
 StopMotors();
 LCD.WriteLine("Robot is against wall.");
 Drive(20, EAST, NORTH);
 Sleep(250);
 
 LCD.WriteLine("Drive up hill.");
 Drive(90, 190, NORTH);
 //    while(Accel.Z() < -.970) {}
 //    LCD.WriteLine("On Hill.");
 //    while(Accel.Z() > -.965) {}
 //    LCD.WriteLine("Off Hill.");
 
 Sleep(2000);
 StopMotors();
 Sleep(1000);
 
 Drive(30, WEST, NORTH);
 
 while(switch7.Value() || switch6.Value()) {}
 
 Drive(50, SOUTH, NORTH);
 
 while(switch5.Value() && switch4.Value()) {}
 
 StopMotors();
 LCD.WriteLine("Robot is against seismograph.");
 
 Drive(10, NORTH, NORTH);
 Sleep(300);
 
 Drive(35, 80, NORTH);
 
 while(switch7.Value() || switch6.Value()){}
 LCD.WriteLine("Robot is against wall.");
 
 Drive(25, NORTH, NORTH);
 Sleep(500);
 
 LCD.WriteLine("Robot is away from seismograph.");
 
 Drive(25, EAST, NORTH);
 
 Sleep(1500);
 LCD.WriteLine("Robot is away from wall.");
 
 Drive(10, 180, NORTH);
 
 Sleep(7000);
 LCD.WriteLine("Robot has hit button.");
 
 Drive(50, NORTHEAST, NORTH);
 Sleep(2500);
 
 LCD.WriteLine("Robot has hit lever area.");
 
 StopMotors();
 LCD.WriteLine("Testing is over.");
 }
 */
