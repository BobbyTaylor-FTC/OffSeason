package org.firstinspires.ftc.teamcode.leagueRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class grabber
{

    LinearOpMode opModeObj;
    public Servo    leftStone    = null;
    public Servo    rightStone   = null;

    public grabber(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap){
        opModeObj = opmode;
        leftStone = hardwareMap.get(Servo.class,"left_stone");
        rightStone = hardwareMap.get(Servo.class,"right_stone");
    }
    public void grabSkystone(int pos){
        if(pos == 1){
            rightStone.setPosition(.95);
        }
        if(pos == 0){
            leftStone.setPosition(0);
        }
    }
    public void releaseSkystone(){
        //don't need to specify which grabber is released because all can be set to go to their 0 position with impunity
        rightStone.setPosition(.45);
        leftStone.setPosition(.5 );
    }


}
