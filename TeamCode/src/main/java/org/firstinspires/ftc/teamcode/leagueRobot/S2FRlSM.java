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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
Start with the front color sensor inline with the middle of the first stone
 */

@Autonomous(name="Two Sample Foundation Red", group="Linear Opmode")

public class S2FRlSM extends LinearOpMode {

    private enum State{
        STATE_INITIAL,
        STATE_DRIVE_TO_STONE,
        STATE_LOCATE_STONE,
        STATE_DRIVE_TO_DUMP,
        STATE_DUMP,
        STATE_RETURN,
        STATE_ACQUIRE_2STONE,
        STATE_DRIVE_TO_LAST_DUMP,
        STATE_PULL_FOUNDATION,
        STATE_PARK,
        STATE_STOP,
    }

    private ElapsedTime mStateTime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    public ColorSensor sensorColorFront = null;
    public ColorSensor sensorColorMiddle = null;
    private State mCurrentState; //Current State Machine State.
    public S2FRlSM(){

    }


    @Override
    public void runOpMode()
    {
        drive vroom = new drive(this, telemetry, hardwareMap);
        color see = new color(this, telemetry, hardwareMap);
        //lift ellie = new lift(this, telemetry, hardwareMap);
        //revIMU gyro = new revIMU(this, telemetry, hardwareMap);
        found pull = new found(this, telemetry, hardwareMap);
        grabber grabby = new grabber(this, telemetry, hardwareMap);
        range scope = new range(this, telemetry, hardwareMap);
        sensorColorMiddle = hardwareMap.get(ColorSensor.class,"Msensor_color_distance");
        sensorColorFront = hardwareMap.get(ColorSensor.class,"Fsensor_color_distance");
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
                    grabby.releaseSkystone();
                    //pull.releaseFound();
                    newState(State.STATE_DRIVE_TO_STONE);
                    break;
                case STATE_DRIVE_TO_STONE:
                    vroom.driveY(27.2, 1, 1.29);
                    vroom.driveX(10, 1, .3);
                    vroom.delay(.2);
                    //vroom.driveX(10,1,1);
                    newState(State.STATE_LOCATE_STONE);
                    //newState(State.STATE_LOCATE_STONE);
                    break;
                case STATE_LOCATE_STONE:
                    //vroom.driveY(2.7, .5, 1);
                    if (see.isSkystone(sensorColorFront, 3))
                    {
                        skyStoneLocation = 0;
                        telemetry.addData("Skystone at ", skyStoneLocation);
                        telemetry.update();
                        //vroom.driveY(8, .5, 1);
                        //skystone grabber code here
                        grabby.grabSkystone(skyStoneLocation);
                        vroom.delay(.5);
                        vroom.driveY(-5.5, 1, 1.3);
                    } else if (see.isSkystone(sensorColorMiddle, 2))
                    {
                        skyStoneLocation = 1;
                        telemetry.addData("Skystone at ", skyStoneLocation);
                        telemetry.update();
                        //vroom.driveY(8, .5, 1);
                        //skystone grabber code here
                        grabby.grabSkystone(skyStoneLocation);
                        vroom.delay(.5);
                        vroom.driveY(-5.5, 1, 1.3);
                    } else
                    {
                        //vroom.driveY(-3, .5, 1);
                        skyStoneLocation = 2;
                        telemetry.addData("Skystone at ", skyStoneLocation);
                        telemetry.update();
                        vroom.driveX(-6, 1, 1);
                        //vroom.driveY(8, .5, 1);
                        //skystone grabber code here
                        grabby.grabSkystone(0);
                        vroom.delay(.5);
                        vroom.driveY(-5.5, 1, 1.3);
                        vroom.driveX(9, 1, 1);

                    }
                    newState(State.STATE_DRIVE_TO_DUMP);
                    break;
                case STATE_DRIVE_TO_DUMP:
                    vroom.turnto(0, 1.2);
                    vroom.driveX(55, 1, 3);
                    vroom.driveY(8,1,1); //this will get it to touch foundation
                    newState(State.STATE_DUMP);
                    break;
                case STATE_DUMP:
                    //skystone grabber release code here
                    vroom.delay(.5);
                    grabby.releaseSkystone();
                    vroom.driveY(-6,1,1);
                    newState(State.STATE_RETURN);
                    break;
                case STATE_RETURN:
                    vroom.turnto(0,1) ;
                    if(skyStoneLocation<2)
                    {
                        vroom.driveX(-86.7 , 1, 2);
                    }
                    else{
                        vroom.driveX(-95  ,1,2);
                    }
                    vroom.turnto(0,1);
                    //vroom.driveX(-45, 1, 3);
                    //vroom.driveY(15  ,.9,.8);
                    //vroom.driveX(5 ,1,1);
                    newState(State.STATE_ACQUIRE_2STONE);
                    break;
                case STATE_ACQUIRE_2STONE:

                    vroom.driveY(5.5, 1, 1);
                    if(skyStoneLocation ==1){
                        grabby.grabSkystone(1);
                    }
                    else{
                        grabby.grabSkystone(0);
                    }
                    //skystone grabber code here
                    //grabby.grabSkystone(0);
                    vroom.turnto(0,1);
                    vroom.driveY(-10.5, 1, 1);
                    newState(State.STATE_DRIVE_TO_LAST_DUMP);
                    break;
                case STATE_DRIVE_TO_LAST_DUMP:
                    vroom.turnto(0,1);
                    if(skyStoneLocation<2)
                    {
                        vroom.driveX(78, 1, 2);
                    }
                    else{
                        vroom.driveY(-1,1,1);
                        vroom.driveX(90,1,2);
                    }
                    vroom.driveY(8  ,.5,1);
                    vroom.driveY(-4.5,1,1);
                    //skystone grabber code here
                    grabby.releaseSkystone();
                    newState(State.STATE_PULL_FOUNDATION);
                    break;
                case STATE_PULL_FOUNDATION:
                    vroom.turnto(-90,1);
                    vroom.driveX(-8.8,.5,1);
                    vroom.turnto(-90,1);
                    pull.grabFound();
                    vroom.delay(.5);
                    vroom.driveX(30,.5,1);
                    pull.releaseFound();
                    vroom.delay(.2);
                    /*vroom.driveY(-40,1,1);
                    vroom.driveX(-20,1,1);
                    vroom.driveY(-20,1,1);
                    */

                    newState(State.STATE_PARK);
                    break;
                case STATE_PARK:
                    //vroom.driveX(-30, 1, 3);
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
