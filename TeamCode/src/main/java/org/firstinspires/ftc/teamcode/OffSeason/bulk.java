package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class bulk
{
    LinearOpMode opModeObj;
    public bulk(LinearOpMode opmode){
        opModeObj = opmode;
        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs =opModeObj.hardwareMap.getAll(LynxModule.class);
        bulkManual(allHubs);
    }

    public void bulkManual(List<LynxModule> myHubs){

        for (LynxModule module : myHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void bulkManualClear() {
        for (LynxModule module : opModeObj.hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }




}
