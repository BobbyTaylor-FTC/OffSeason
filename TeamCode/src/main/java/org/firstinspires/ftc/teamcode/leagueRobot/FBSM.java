/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.leagueRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
Start with the front color sensor inline with the middle of the first stone
 */

@Autonomous(name="Foundation Blue", group="Linear Opmode")

public class FBSM extends LinearOpMode {

    private enum State{
        STATE_INITIAL,
        STATE_DRIVE_TO_FOUND,
        STATE_GRAB_FOUND,
        STATE_PULL_FOUND,
        STATE_RELEASE_FOUND,
        STATE_STOP,
        STATE_PARK,
    }

    private ElapsedTime mStateTime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    private State mCurrentState; //Current State Machine State.
    public FBSM(){

    }


    @Override
    public void runOpMode()
    {
        drive vroom = new drive(this, telemetry, hardwareMap);
        color see = new color(this, telemetry, hardwareMap);
        lift ellie = new lift(this, telemetry, hardwareMap);
        revIMU gyro = new revIMU(this, telemetry, hardwareMap);
        found pull = new found(this, telemetry, hardwareMap);
        grabber grabby = new grabber(this, telemetry, hardwareMap);
        range scope = new range(this, telemetry, hardwareMap);
        bulk reader = new bulk(this,telemetry,hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();
        mStateTime.reset();
        newState(State.STATE_INITIAL);  //newState(State.expample); changes state

        //Execute the current state. Each STATE's case code does the following:
        //1: Look for an EVENT that wil cause a STATE change
        //2: If an EVENT is found, take any required ACTION, an then set the next STATE else
        //3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE


        int skyStoneLocation = 0; //location of skystone; 0: 1st position, 1: 2nd position, 2: 3rd position
        while (!isStopRequested() && opModeIsActive())
        {
            switch (mCurrentState)
            {
                case STATE_INITIAL:
                    pull.releaseFound();
                    vroom.delay(.5);
                    newState(State.STATE_DRIVE_TO_FOUND);
                    break;
                case STATE_DRIVE_TO_FOUND:
                    vroom.driveX(-23,1,3);
                    vroom.driveY(-8,.8,1);
                    vroom.driveX(-7,.3,1);
                    newState(State.STATE_GRAB_FOUND);
                    break;
                case STATE_GRAB_FOUND:
                    pull.grabFound();
                    vroom.delay(.5);
                    newState(State.STATE_PULL_FOUND);
                    break;
                case STATE_PULL_FOUND:
                    vroom.driveX(33,1,1);
                    newState(State.STATE_RELEASE_FOUND);
                    break;
                case STATE_RELEASE_FOUND:
                    pull.releaseFound();
                    vroom.delay(.5);
                    newState(State.STATE_PARK);
                    break;
                case STATE_PARK:
                    vroom.driveY(32,1,3);
                    vroom.driveX(-23,.8,1);
                    vroom.driveY(-10,.3,1);
                    vroom.driveY(25,1,1);
                    newState(State.STATE_STOP);
                    break;
                case STATE_STOP:

                    break;
            }

        }
    }
    private void newState(State newState){
        mStateTime.reset();
        mCurrentState = newState;
    }



}
