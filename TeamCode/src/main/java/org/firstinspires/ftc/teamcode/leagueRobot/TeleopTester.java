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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
Start with the front color sensor inline with the middle of the first stone
 */

@TeleOp(name="FantanaEX", group="Linear Opmode")

public class TeleopTester extends LinearOpMode
{


    private ElapsedTime runtime = new ElapsedTime();

    boolean shouldLiftMove = true; // if this variable is true, the lift will move to the position dictated by the gamepads.
    //if the variable is false, the lift will not move to the position and will instead stay at a position dictated by the right gamestick

    public DcMotor leftLift = null;
    public DcMotor rightLift = null;



    @Override
    public void runOpMode()
    {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        runtime.reset();

        drive vroom = new drive(this,telemetry,hardwareMap);
        color see = new color(this,telemetry,hardwareMap);
        lift ellie = new lift(this,telemetry,hardwareMap);
        revIMU gyro = new revIMU(this,telemetry,hardwareMap);
        found pull = new found(this,telemetry,hardwareMap);
        grabber grabby = new grabber(this,telemetry,hardwareMap);
        range scope = new range(this,telemetry,hardwareMap);
        leftLift = hardwareMap.get(DcMotor.class,"left_lift");
        rightLift = hardwareMap.get(DcMotor.class,"right_lift");
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setPower(0);
        rightLift.setPower(0);
        int liftLocation = 0; //height of lift: 0 = ground, 1 = first stone, 2 = second stone, 3 = third stone, 4 = fourth stone
        while (opModeIsActive())
        {


//lift code
                if(gamepad2.dpad_up){
                    if(liftLocation<ellie.maxHeight){ //makes sure that the lift cannot get past the maximum height which would likely break the lift
                        liftLocation++;
                        telemetry.addData("Going up", "");

                    }
                    shouldLiftMove = true;
                }
                else if(gamepad2.dpad_down){
                    if(liftLocation>0){ //makes sure that the lift cannot get into a negative position which would likely break the lift
                        liftLocation--;
                        telemetry.addData("Going down", "");
                    }
                    shouldLiftMove = true;
                }
                else if(gamepad2.dpad_right){
                    liftLocation = ellie.maxHeight; //replace with max lift position
                    shouldLiftMove = true;
                    telemetry.addData("Max height", "");
                }
                else if(gamepad2.dpad_left){
                    liftLocation = 0; //min lift position
                    shouldLiftMove = true;
                    telemetry.addData("Min height", "");
                }



                if(gamepad2.left_stick_y!=0){
                    if((ellie.maxHeight*ellie.ticksPerHeight>(leftLift.getCurrentPosition()+20))&&leftLift.getCurrentPosition()-20>0){
                        ellie.setLiftPower(gamepad2.left_stick_y);
                    }

                    if((ellie.maxHeight*ellie.ticksPerHeight>leftLift.getCurrentPosition()+50)&&(0<leftLift.getCurrentPosition()+30)) {
                        ellie.setLiftPower(gamepad1.left_stick_y);
                    }
                    else if(ellie.maxHeight*ellie.ticksPerHeight>leftLift.getCurrentPosition()+50){
                        if(gamepad2.left_stick_y>0){
                            ellie.setLiftPower(gamepad2.left_stick_y);
                        }
                    }
                    else if(0<leftLift.getCurrentPosition()+30){
                        if(gamepad2.left_stick_y<0){
                            ellie.setLiftPower(gamepad2.left_stick_y);
                        }
                    }
                telemetry.addData("Custom movement", gamepad2.left_stick_y);
                shouldLiftMove = false;
                }
                else if(shouldLiftMove){
                    ellie.liftMove(liftLocation);
                    telemetry.addData("Lift moving to", liftLocation);
                }
                telemetry.addData("Should the lift move?",shouldLiftMove);



                if(gamepad1.a){
                    pull.grabFound();
                }
                if(!gamepad1.a){
                    pull.releaseFound();
                }

                if(gamepad1.b){
                    grabby.grabSkystone(0);
                    grabby.grabSkystone(1);
                }
                else {
                    grabby.releaseSkystone();
                }



                //drive code
                    vroom.driveT(1);
                /*
                                if(gamepad2.b){
                    vroom.turn(90);
                }
                 */

                if(gamepad2.x){
                    telemetry.addData("How far",scope.range());
                }
                telemetry.addData("Lift Left encoders: ",leftLift.getCurrentPosition());
                telemetry.addData("Lift Left power: ",leftLift.getPower());
                telemetry.addData("Lift Right encoders: ",rightLift.getCurrentPosition());
                telemetry.addData("Lift Right power: ",rightLift.getPower());

            telemetry.update();
            }

        }
    }
