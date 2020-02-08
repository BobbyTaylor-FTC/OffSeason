package org.firstinspires.ftc.teamcode.leagueRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class color
{


    LinearOpMode opModeObj;
    ColorSensor sensorColor;
    public ColorSensor sensorColorFront = null;
    public ColorSensor sensorColorMiddle = null;

    public color(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap){
        opModeObj = opmode;
        sensorColorMiddle = hardwareMap.colorSensor.get("Msensor_color_distance");
        sensorColorFront = hardwareMap.get(ColorSensor.class, "Fsensor_color_distance");
    }

    public boolean isSkystone(ColorSensor sense, int cc){//cc is 2 for color sensor v2, and 3 for color sensor v3
        sensorColor = sense;
        try{
            if((sense.red() * sense.green() / (sense.blue() * sense.blue()))<=cc){
                opModeObj.telemetry.addData("skystone found", "found");
                opModeObj.telemetry.update();
                return true;
            }
            else {
                opModeObj.telemetry.addData("skystone lost", "lost");
                return false;
            }

        }
        catch(ArithmeticException e){
            if(sense.red() *sense.green() / (.00001 * .00001)<=cc){
                opModeObj.telemetry.addData("skystone found", "found");
                opModeObj.telemetry.update();
                return true;
            }
            else {
                opModeObj.telemetry.addData("skystone lost", "lost");
                opModeObj.telemetry.update();
                return false;
            }
        }
    }


}
