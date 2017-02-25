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
#include <FEHMotor.h>
#include <FEHServo.h>
#define TURN_SAT 0
#define PUSH_BTN 1
#define PULL_LEVER 2
#define EXTRACT_CORE 3
#define DUMP_CORE 4
#define PUSH_END_BTN 5

//Movement Methods
void move(int power, int x, int y) {

}

void rotate(int degrees) {

}

void stopMotors() {

}

void AllMotors(int percent) {

}

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


int main(int argc, const char * argv[]) {
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

}
