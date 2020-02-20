package org.firstinspires.ftc.teamcode.leagueRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class found
{

    LinearOpMode opModeObj;
    public Servo    leftFound    = null;
    public Servo    rightFound   = null;

    public found(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap){
        opModeObj = opmode;
        leftFound = hardwareMap.get(Servo.class,"left_found");
        rightFound = hardwareMap.get(Servo.class,"right_found");

    }
    public void grabFound(){
        rightFound.setPosition(1);
        leftFound.setPosition(.5);
    }
    public void releaseFound(){
        rightFound.setPosition(.5);
        leftFound.setPosition(1);
    }


}
