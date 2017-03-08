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


#define TURN_SAT 0
#define PUSH_BTN 1
#define PULL_LEVER 2
#define EXTRACT_CORE 3
#define DUMP_CORE 4
#define PUSH_END_BTN 5

//Movement Methods

void MoveAlongWall(int directionWall, int directionMove){
    
}


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

void PushButton() {
    
}

void ExtractCore(){
    
}
void DumpCore(){
    
}

void PullLever() {
    
}
void TurnSatellite() {
    
}
void PushEndButton() {
    
}

void ResetTask(struct Task){
    
}

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

/*
 float DriveOnLine(int motorPercent);
 {
 if(lLine.Value() && cLine.Value() && rLine.Value()) {
 Drive(motorPercent, SOUTHWEST, NORTHWEST);
 } else if(rLine.Value() && lLine.Value()) {   //Right and left side are messed up
 StopMotors();
 LCD.WriteLine("Shit is fucked.");
 } else if(rLine.Value()) {  //Right side isn't providing input
 Drive(motorPercent, SOUTH, NORTHWEST);
 } else if (lLine.Value()) {   //Left side isn't providing input
 Drive(motorPercent, WEST, NORTHWEST);
 }
 else {
 LCD.WriteLine("Shit is suppppperrr fucked.");
 }
 }
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
        int power = (degRem / 9 + 7) * modifier;
        AllMotors(power);
    }
    StopMotors();
}

void testRotate() {
    LCD.Write(RPS.Heading());
    Sleep(1000);
    Rotate(SOUTH);
    SD.Printf("Should be 180 is %f\n", RPS.Heading());
    LCD.Write("Facing South.");
    Sleep(1000);
    Rotate(WEST);
    SD.Printf("Should be 90 is %f\n", RPS.Heading());
    LCD.Write("Facing West.");
    Sleep(1000);
    Rotate(SOUTHWEST);
    SD.Printf("Should be 135 is %f\n", RPS.Heading());
    LCD.Write("Facing Southwest.");
    Sleep(1000);
    Rotate(NORTH);
    SD.Printf("Should be 0 is %f\n", RPS.Heading());
    LCD.Write("Facing North.");
    Sleep(1000);
    Rotate(EAST);
    SD.Printf("Should be 270 is %f\n", RPS.Heading());
    LCD.Write("Facing East.");
    Sleep(1000);
    Rotate(NORTHWEST);
    SD.Printf("Should be 45 is %f\n", RPS.Heading());
    LCD.Write("Facing Northwest.");
    Sleep(1000);
}

int main(void) {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    RPS.InitializeTouchMenu();
    SD.OpenLog();
    testRotate();
    SD.CloseLog();
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
