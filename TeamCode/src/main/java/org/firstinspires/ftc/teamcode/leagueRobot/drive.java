package org.firstinspires.ftc.teamcode.leagueRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class drive
{
    static final double COUNTS_PER_MOTOR_REV = 537.5;    // eg: Hex HD Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    LinearOpMode opModeObj;


    //variables for taking the derivative of error
    public static double lasterror;
    public double error;
    public double output;
    public double scaledoutput;
    public double nowtime, thentime;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime motortime = new ElapsedTime();


    public double kPturn = .03;
    public double kDturn = 0;

    public double kPstrafeangle = 0.3;
    public double kDstrafeangle = 0;

    public double kPstrafe = .03;
    public double kDstrafe = 0;

    public double kPdrive = 0.1;
    public double kIdrive = 0;
    public double kDdrive = 0;

    public double strafeHeading = 0;

    public int frontLefttarget, frontRighttarget,backRighttarget,backLefttarget;

    private int error1, error2, error3, error4, lasterror1,lasterror2,lasterror3,lasterror4;
    private int integral1 = 0, integral2 = 0, integral3 = 0, integral4 = 0;
    private double output1, output2, output3, output4;

    public DcMotorEx frontLeft   = null;
    public DcMotorEx  frontRight  = null;
    public DcMotorEx  backLeft   = null;
    public DcMotorEx  backRight  = null;
    public revIMU gyro;
    public range scope;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

public drive(LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap){
    this.hardwareMap = hardwareMap;
    opModeObj = opmode;
    frontLeft  = hardwareMap.get(DcMotorEx.class,"front_left");
    frontRight = hardwareMap.get(DcMotorEx.class,"front_right");
    backLeft    =hardwareMap.get(DcMotorEx.class,"back_left");
    backRight    = hardwareMap.get(DcMotorEx.class,"back_right");
    frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    // Set all motors to zero power
    frontLeft.setPower(0);
    frontRight.setPower(0);
    backLeft.setPower(0);
    backRight.setPower(0);
    gyro = new revIMU(opModeObj,telemetry,hardwareMap);
    scope = new range(opModeObj,telemetry,hardwareMap);

    }

    public void driveT(double percentSpeed){
        double drive = -this.opModeObj.gamepad1.left_stick_y; // inputs
        double strafe = -this.opModeObj.gamepad1.left_stick_x;
        double turn = this.opModeObj.gamepad1.right_stick_x;

        double lfP = drive + strafe - turn; // determining wheel proportions
        double lbP = drive - strafe - turn;
        double rfP = drive - strafe + turn;
        double rbP = drive + strafe + turn;

        double max = Math.max(1.0, Math.abs(lfP)); // smooth out a little - can be deleted
        max = Math.max(max, Math.abs(lbP));
        max = Math.max(max, Math.abs(rfP));
        max = Math.max(max, Math.abs(rbP));
        if(this.opModeObj.gamepad1.left_bumper){
            strafeLeftT(1);
        }
        else if(this.opModeObj.gamepad1.right_bumper){
            strafeRightT(1);
        }
        else if(this.opModeObj.gamepad1.left_trigger!=0){
            gyroStrafeTele(-1);
        }
        else if(this.opModeObj.gamepad1.right_trigger!=0){
            gyroStrafeTele(1);
        }
        else{
            frontLeft.setPower(lfP * percentSpeed); // set powers
            backLeft.setPower(lbP * percentSpeed);
            frontRight.setPower(rfP * percentSpeed);
            backRight.setPower(rbP * percentSpeed);
          //  strafeHeading = gyro.getAngle();
        }
    }
    public void strafeLeftT(double percentSpeed){
        setPowerY(percentSpeed);
    }
    public void strafeRightT(double percentSpeed){
        setPowerY(-percentSpeed);
    }


    public void driveX(double x, double speed, double time)
    {
        x = (int) x * COUNTS_PER_INCH;
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLefttarget = (int) (frontLeft.getCurrentPosition() + x);
        opModeObj.telemetry.addData("where I meant to be at", frontLefttarget);

        frontLeft.setTargetPosition(frontLefttarget);
        frontRight.setTargetPosition(frontLefttarget);
        backRight.setTargetPosition(frontLefttarget);
        backLeft.setTargetPosition(frontLefttarget);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        setPower(speed);
        isBusy(time);
        setPower(0);
                frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        // Ensure that the opmode is still active

    }

    public void driveY(double y, double speed, double time)
    {

        y = (int) y * COUNTS_PER_INCH;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLefttarget = (int) (frontLeft.getCurrentPosition() + y);
        frontRighttarget = (int) (frontRight.getCurrentPosition() - y);
        backRighttarget = (int) (backRight.getCurrentPosition() + y);
        backLefttarget = (int) (backLeft.getCurrentPosition() - y);

        frontLeft.setTargetPosition(frontLefttarget);
        frontRight.setTargetPosition(frontRighttarget);
        backRight.setTargetPosition(backRighttarget);
        backLeft.setTargetPosition(backLefttarget);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(speed);
        isBusy(time);
        setPower(0);
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }





    /*
    public void driveX(double x, double speed, double time)
    {
        x = (int) x * COUNTS_PER_INCH;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLefttarget = (int) (frontLeft.getCurrentPosition() + x);
        this.opModeObj.telemetry.addData("where I meant to be at", frontLefttarget);
        frontLeft.setTargetPosition(frontLefttarget);
        frontRight.setTargetPosition(frontLefttarget);
        backRight.setTargetPosition(frontLefttarget);
        backLeft.setTargetPosition(frontLefttarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
this.opModeObj.telemetry.addData("pre PID loop","");
this.opModeObj.telemetry.update();
        motortime.reset();
        setPowerX(1);
        this.opModeObj.telemetry.addData("frontLeft is busy?",frontLeft.isBusy());
        this.opModeObj.telemetry.addData("motortime",motortime.seconds());
        this.opModeObj.telemetry.update();
        while ((frontLeft.getPower()!=0||frontRight.getPower()!=0 || backLeft.getPower()!=0 || backRight.getPower()!=0)&&motortime.seconds()<time)
        {
            frontLeft.getVelocity()
            PIDDrive(speed);

            this.opModeObj.telemetry.addData("Where I at fL", frontLeft.getCurrentPosition());
            this.opModeObj.telemetry.addData("Power fL",frontLeft.getPower());
            this.opModeObj.telemetry.addData("Where I at fR", frontRight.getCurrentPosition());
            this.opModeObj.telemetry.addData("Power fR",frontRight.getPower());
            this.opModeObj.telemetry.addData("Where I at bL", backLeft.getCurrentPosition());
            this.opModeObj.telemetry.addData("Power bL",backLeft.getPower());
            this.opModeObj.telemetry.addData("Where I at bR", backRight.getCurrentPosition());
            this.opModeObj.telemetry.addData("Power bR",backRight.getPower());

            this.opModeObj.telemetry.update();
        }
        setPower(0);
    }

    public void driveY(double y, double speed, double time)
    {

        y = (int) y * COUNTS_PER_INCH;
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLefttarget = (int) (frontLeft.getCurrentPosition() + y);
        frontRighttarget = (int) (frontRight.getCurrentPosition() - y);
        backRighttarget = (int) (backRight.getCurrentPosition() + y);
        backLefttarget = (int) (backLeft.getCurrentPosition() - y);

        frontLeft.setTargetPosition(frontLefttarget);
        frontRight.setTargetPosition(frontRighttarget);
        backRight.setTargetPosition(backRighttarget);
        backLeft.setTargetPosition(backLefttarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motortime.reset();
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()&&motortime.seconds()<time)
        {
            PIDDrive(speed);

            this.opModeObj.telemetry.addData("Where I at fL", frontLeft.getCurrentPosition());
            this.opModeObj.telemetry.addData("Where I at fR", frontRight.getCurrentPosition());
            this.opModeObj.telemetry.addData("Where I at bL", backLeft.getCurrentPosition());
            this.opModeObj.telemetry.addData("Where I at bR", backRight.getCurrentPosition());

            this.opModeObj.telemetry.update();
        }

    }
*/
    public void gyroStrafeTele(double speed){ //check to see if this is the correct code
            nowtime = runtime.seconds();
            error = gyro.getAngle()-strafeHeading;
            double output = kPstrafeangle*error + kDstrafeangle*(error-lasterror)/(nowtime-thentime);

            double frp = -speed+output;
            double flp = speed+output;

            if(Math.abs(frp)>1||Math.abs(flp)>1){
                double frpN=frp/Math.max(frp,flp);
                double flpN=flp/Math.max(frp,flp);
                frontLeft.setPower(flpN);
                frontRight.setPower(frpN);
                backLeft.setPower(frpN);
                backRight.setPower(flpN);
            }
            else{
                frontLeft.setPower(flp);
                frontRight.setPower(frp);
                backLeft.setPower(frp);
                backRight.setPower(flp);
            }



            //store these variables for the next loop
            lasterror = error;
            thentime = nowtime;
    }



    public void setPower(double speed)
    {
        frontRight.setPower(speed);
        frontLeft.setPower(speed);
        backRight.setPower(speed);
        backLeft.setPower(speed);
    }
    public void setPowerX(double speed)
    {
        frontRight.setPower(speed);
        frontLeft.setPower(speed);
        backRight.setPower(speed);
        backLeft.setPower(speed);
    }
    public void setPowerY(double speed)
    {
        frontRight.setPower(-speed);
        frontLeft.setPower(speed);
        backRight.setPower(speed);
        backLeft.setPower(-speed);
    }
    public void setPowerT(double speed){
        frontRight.setPower(-speed);
        frontLeft.setPower(speed);
        backRight.setPower(-speed);
        backLeft.setPower(speed);
    }


    public void turn(double degrees){
        double startHeading = gyro.getAngle();
        while(posDif(gyro.getAngle(),startHeading+degrees)>5){
            //blocking code because the myMap will 100% need to finish turning
            nowtime = runtime.seconds();
            error = gyro.getAngle()-startHeading+degrees;
            double output = kPturn*error + kDturn*(error-lasterror)/(nowtime-thentime);
            setPowerT(output);


            //store these variables for the next loop
            lasterror = error;
            thentime = nowtime;
        }
    }
    public void strafeToSkystone(){
        while(scope.range()>1){
            nowtime = runtime.seconds();
            error = scope.range()-30;
            double output = kPstrafe*error + kDstrafe*(error-lasterror)/(nowtime-thentime);
            gyroStrafeTele(output);
            //store these variables for the next loop
            lasterror = error;
            thentime = nowtime;
        }

    }

    public void isBusy(double time)
    {
        ElapsedTime motortime = new ElapsedTime();
        motortime.reset();
        while ((frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())&&(motortime.seconds()<time))
        {
            this.opModeObj.telemetry.addData("Where I at fL", frontLeft.getCurrentPosition());
            this.opModeObj.telemetry.addData("How fast?", frontLeft.getPower());
            this.opModeObj.telemetry.addData("Where I at fR", frontRight.getCurrentPosition());
            this.opModeObj.telemetry.addData("How fast?", frontRight.getPower());
            this.opModeObj.telemetry.addData("Where I at bL", backLeft.getCurrentPosition());
            this.opModeObj.telemetry.addData("How fast?", backLeft.getPower());
            this.opModeObj.telemetry.addData("Where I at bR", backRight.getCurrentPosition());
            this.opModeObj.telemetry.addData("How fast?", backRight.getPower());

            this.opModeObj.telemetry.update();
        }
    }

    public void delay(double time){
        ElapsedTime delaytime = new ElapsedTime();
        delaytime.reset();
        while(delaytime.seconds()<time){
        }
    }
    public void PIDDrive(double speed){
        nowtime = runtime.seconds();
        error1 = -frontLeft.getCurrentPosition()+frontLeft.getTargetPosition();
        error2 = -frontRight.getCurrentPosition()+frontRight.getTargetPosition();
        error3 = -backLeft.getCurrentPosition()+backLeft.getTargetPosition();
        error4 = -backRight.getCurrentPosition()+backRight.getTargetPosition();
        integral1 += error1;
        integral2 += error2;
        integral3 += error3;
        integral4 += error4;
        output1 = kPdrive*error1 + kDdrive*(error1-lasterror1)/(nowtime-thentime)+kIdrive*integral1;
        output2 = kPdrive*error2 + kDdrive*(error2-lasterror2)/(nowtime-thentime)+kIdrive*integral2;
        output3 = kPdrive*error3 + kDdrive*(error3-lasterror3)/(nowtime-thentime)+kIdrive*integral3;
        output4 = kPdrive*error4 + kDdrive*(error4-lasterror4)/(nowtime-thentime)+kIdrive*integral4;
        frontLeft.setPower(output1);
        frontRight.setPower(output2);
        backLeft.setPower(output3);
        backRight.setPower(output4);
        //store these variables for the next loop
        lasterror1 = error1;
        lasterror2 = error2;
        lasterror3 = error3;
        lasterror4 = error4;

        
        thentime = nowtime;
    }


    public int posDif(int currentPos,int goal){
        return Math.abs(currentPos-goal);
    }
    public double posDif(double currentPos,double goal){
        return Math.abs(currentPos-goal);
    }
    public boolean moveComplete(){
        if((Math.abs(frontLeft.getCurrentPosition()-frontLefttarget))<30||(Math.abs(frontLeft.getCurrentPosition()-frontLefttarget))<30
                ||(Math.abs(frontLeft.getCurrentPosition()-frontLefttarget))<30||(Math.abs(frontLeft.getCurrentPosition()-frontLefttarget))<30){
            return true;
        }
        return false;
    }

}
