#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHAccel.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <math.h>

//GLOBALS

//MOTORS
FEHMotor fLeft(FEHMotor::Motor0, 12.0);
FEHMotor fRight(FEHMotor::Motor1, 12.0);
FEHMotor bLeft(FEHMotor::Motor2, 12.0);
FEHMotor bRight(FEHMotor::Motor3, 12.0);

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
//void Drive(int power, int x, int y) {
//    float direction = RPS.Heading();
//    float curX = RPS.X();
//    float curY = RPS.Y();

//}

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

//void Rotate(int degree) {
//    StopMotors();
//    if(abs(RPS.Heading() - (float)degree) < 180) {
//        AllMotors(50);
//    }
//    else {
//        AllMotors(-50);
//    }
//    while(abs(RPS.Heading() - (float)degree) > 3) {}
//    StopMotors();
//}

int main(void)
{
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

