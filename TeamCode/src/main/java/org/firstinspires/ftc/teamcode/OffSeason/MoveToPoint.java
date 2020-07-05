package org.firstinspires.ftc.teamcode.OffSeason;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.OffSeason.Point.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Array;
import java.util.ArrayList;

import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.OffSeason.MathFunctions.AngleWrap;


public class MoveToPoint {

    public static Odometry pure;
    public static DcMotorEx frontLeft   = null;
    public static DcMotorEx  frontRight  = null;
    public static DcMotorEx  backLeft   = null;
    public static DcMotorEx  backRight  = null;

    public MoveToPoint(Odometry missile, LinearOpMode opmode, Telemetry telemetry, HardwareMap hardwareMap){
    pure = missile;
        frontLeft  = hardwareMap.get(DcMotorEx.class,"front_left");
        frontRight = hardwareMap.get(DcMotorEx.class,"front_right");
        backLeft = hardwareMap.get(DcMotorEx.class,"back_left");
        backRight = hardwareMap.get(DcMotorEx.class,"back_right");
    }

    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(pure.getGlobalPositionX(),pure.getGlobalPositionY()), allPoints.get(0).followDistance);
        toPoint(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i<pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation,followRadius,startLine.toPoint(), endLine.toPoint());


            double closestAngle = 10000000;

            for(Point thisIntersection : intersections){
                double angle = atan2(thisIntersection.y - pure.getGlobalPositionY(), thisIntersection.x - pure.getGlobalPositionX());
                double deltaAngle = abs(MathFunctions.AngleWrap(angle-pure.getGlobalPositionTheta()));

                if(deltaAngle<closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    public static void toPoint(double x, double y, double movementSpeed, double preferredAngle, double turnspeed){

        double absoluteAngleToTarget = atan2(y-pure.getGlobalPositionY(), x-pure.getGlobalPositionX());

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - pure.getGlobalPositionTheta());

        double totalDistance = sqrt(pow(x-pure.getGlobalPositionX(),2)+pow(y-pure.getGlobalPositionY(),2));

        double relativeXToPoint = cos(relativeAngleToPoint)*totalDistance;
        double relativeYToPoint = sin(relativeAngleToPoint)*totalDistance;

        double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));

       double movement_x = movementXPower*movementSpeed;
       double movement_y =  movementYPower*movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint +preferredAngle;
        double movement_turn = Range.clip(relativeTurnAngle/toRadians(30),-1,1)*turnspeed;

        if(totalDistance < 10){
            movement_turn = 0;
        }
        frontLeft.setPower(movement_y + movement_x - movement_turn);
        frontRight.setPower(movement_y - movement_x + movement_turn);
        backLeft.setPower(movement_y - movement_x - movement_turn);
        backRight.setPower(movement_y + movement_x +movement_turn);
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
        if(abs(linePoint1.y-linePoint2.y)<.003){

            linePoint1.y = linePoint2.y+.0003;
        }
        if(abs(linePoint1.x-linePoint2.x)<.0003) {
            linePoint1.x = linePoint2.x+.0003;
        }
        double m1 = (linePoint2.y-linePoint1.y)/(linePoint2.x-linePoint1.x);

        double quadraticA = 1.0+pow(m1,2);
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1)-(2.0*pow(m1,2)*x1);

        double quadraticC = ((pow(m1,2)*pow(x1,2))) - (2.0*y1*m1*x1) + pow(y1,2) - pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try{
            double xRoot1 = (-quadraticB - sqrt(pow(quadraticB,2)-4.0*quadraticA*quadraticC))/(2.0*quadraticA);

            double yRoot1 = m1 * (xRoot1-x1)+y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint1.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint1.x ? linePoint1.x : linePoint2.x;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));

            }
            double xRoot2 = (-quadraticB + sqrt(pow(quadraticB,2)-4.0*quadraticA*quadraticC))/(2.0*quadraticA);
            double yRoot2 = m1 * (xRoot2-x1)+y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));

            }
        }catch (Exception e){

        }
        return allPoints;


    }

}
