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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
Start with the front color sensor inline with the middle of the first stone
 */
@Disabled
@TeleOp(name="SampleCheck", group="Linear Opmode")

public class Sampling extends LinearOpMode
{

    public ColorSensor sensorColorFront = null;
    public ColorSensor sensorColorMiddle = null;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lastLoop = new ElapsedTime();

    boolean shouldLiftMove = true; // if this variable is true, the lift will move to the position dictated by the gamepads.
    //if the variable is false, the lift will not move to the position and will instead stay at a position dictated by the right gamestick

    boolean grabFound = false;
    boolean grabStone = false;

    boolean shouldHoldHeight = false;

    int holdPosition;

    public DcMotor leftLift = null;
    public DcMotor rightLift = null;



    @Override
    public void runOpMode()
    {
        drive vroom = new drive(this,telemetry,hardwareMap);
        color see = new color(this,telemetry,hardwareMap);
        lift ellie = new lift(this,telemetry,hardwareMap);
        revIMU gyro = new revIMU(this,telemetry,hardwareMap);
        found pull = new found(this,telemetry,hardwareMap);
        grabber grabby = new grabber(this,telemetry,hardwareMap);
        range scope = new range(this,telemetry,hardwareMap);
        claw pince = new claw(this,telemetry,hardwareMap);
        sensorColorMiddle = hardwareMap.get(ColorSensor.class,"Msensor_color_distance");
        sensorColorFront = hardwareMap.get(ColorSensor.class,"Fsensor_color_distance");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        lastLoop.reset();




        int liftLocation = 0; //height of lift: 0 = ground, 1 = first stone, 2 = second stone, 3 = third stone, 4 = fourth stone 5 = fifth stone or capstone
        while (opModeIsActive())
        {
            int skyStoneLocation =0;
    if (see.isSkystone(sensorColorFront, 3))
    {
        skyStoneLocation = 0;
        //skystone grabber code here
        grabby.grabSkystone(skyStoneLocation);
        telemetry.addData("Front sensor","found skystone");
    } else if (see.isSkystone(sensorColorMiddle, 2))
    {
        skyStoneLocation = 1;
        grabby.grabSkystone(skyStoneLocation);
        telemetry.addData("Back sensor","found skystone");
    }
else{
    grabby.releaseSkystone();
}




            telemetry.update();
            }

        }
    }
