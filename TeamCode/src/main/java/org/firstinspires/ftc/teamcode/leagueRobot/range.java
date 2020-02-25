package org.firstinspires.ftc.teamcode.leagueRobot;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
public class range
{

    LinearOpMode opModeObj;
    public DistanceSensor sensorRange;

    public range(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap){
        opModeObj = opmode;
        sensorRange = hardwareMap.get(Rev2mDistanceSensor.class,"sensor_range");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
    }

public double range(){
        return sensorRange.getDistance(DistanceUnit.INCH);
}
}
