package org.firstinspires.ftc.teamcode.OffSeason;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.lang.reflect.Array;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.OffSeason.Point;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

@Disabled
public class PathGenerator {
    public ArrayList<Point> skeletonPath;
    public ArrayList<Point> filledPath;
    public ArrayList<Point> smoothedPath;

    public static double maxPathVelo = 1.52;
    public static double maxPathAccel = .762;
    public static double kConstant = 2;

    public PathGenerator(){
        skeletonPath = new ArrayList<Point>();
        filledPath = new ArrayList<Point>();
        smoothedPath = new ArrayList<Point>();
    }

    /**
     *
     * @param newPoint For the point that we are adding to the path. Points are reached in order of entry
     */
    public void addPoint(Point newPoint){
        skeletonPath.add(newPoint);
    }

    public void fillSmooth(){
        smoother(filler(2),5,5,5);
    }

    public ArrayList<Point> filler(double spacing){
        int originalSize = skeletonPath.size();
        for(int i = 0; i<originalSize-2; i++){
            Vector2d tempVector = new Vector2d(skeletonPath.get(i+1).x-skeletonPath.get(i).x,
                    skeletonPath.get(i+1).y-skeletonPath.get(i).y);
            int pointsThatFit = (int)Math.ceil(tempVector.magnitude()/spacing);
            tempVector = scaling*tempVector.div(tempVector.magnitude());
            for(int n = 0; n<pointsThatFit; n++){
                filledPath.add(skeletonPath.get(i).plus(tempVector.times(n)));
                tempVector = tempVector.div(n);
            }
        }
        filledPath.add(filledPath.get(filledPath.size()-1));
        return filledPath;
    }
    public ArrayList<Point> smoother(){
      /*
      quintic splines go brr
       */
    }

    public void reverseList(){
        ArrayList<Point> tempRevList = new ArrayList<Point>();
        for(int i = (skeletonPath.size()-1); i>-1;i--){
            tempRevList.add(skeletonPath.get(i));
        }
        for(int i = 0; i<tempRevList.size()-1; i++){
            skeletonPath.set(i, tempRevList.get(i));
        }
    }
    public ArrayList<Double> reverseList(ArrayList<Double> origList){
        ArrayList<Double> tempRevList = new ArrayList<Double>();
        for(int i = (origList.size()-1); i>-1;i--){
            tempRevList.add(origList.get(i));
        }
        for(int i = 0; i<tempRevList.size()-1; i++){
            origList.set(i, tempRevList.get(i));
        }
    }

    public double distanceBetweenPoints(ArrayList<Point> givenArray, int pointIndex){
        double totalDistance = 0;
        for(int i = 1; i<pointIndex+1; i++){
            totalDistance += Math.sqrt(
                    Math.pow(givenArray.get(i).x-givenArray.get(i-1).x,2)
                            + Math.pow(givenArray.get(i).y-givenArray.get(i-1).y,2));
        }
        return totalDistance;
    }

    public double curvatureBetweenPoints(Point one, Point two, Point three){
        if(one.x==two.x){
            one.x+=.00001;
        }
        double k1 = 0.5*(Math.pow(one.x,2)+Math.pow(one.y,2)-Math.pow(two.x,2)-Math.pow(two.y,2)/(one.x-two.x);
        double k2 = (one.y-two.y)/(one.x-two.x);
        double b = .5*(Math.pow(two.x,2)-2*two.x*k1+Math.pow(two.y,2)-Math.pow(three.x,2)+2*three.x*k1-Math.pow(three.y,2))/
                (three.x*k2-three.y+two.y-two.x*k2);
        double a = k1-k2*b;

        double r = Math.sqrt(Math.pow(one.x-a,2)+(Math.pow(one.y-b,2)));
        double curvature = 1/r;
        if(Double.isNaN(curvature)){
            return 0;
        }
        return curvature;

    }

    public ArrayList<Double> targetVeloAtI(ArrayList<Point> givenArray){
        if(givenArray.size()<3){
            //Insert error handling code here
        }
        ArrayList<Double> targetVelo = new ArrayList<Double>();
        double distance = distanceFormula(givenArray.get(givenArray.size()-1),givenArray.get(givenArray.size()-2));
        double oldTargetVelo = Math.min(maxPathVelo,kConstant/.0001);
        double newTargetVelo;
        targetVelo.add(oldTargetVelo);
        for(int i = 1; i<givenArray.size()-2; i--){
            distance = distanceFormula(givenArray.get(givenArray.size()-i),givenArray.get(givenArray.size()-i-1));
            oldTargetVelo = Math.min(maxPathVelo,kConstant/curvatureBetweenPoints(givenArray.get(givenArray.size()-i),givenArray.get(givenArray.size()-i-1),givenArray.get(givenArray.size()-i-2)));
            newTargetVelo = Math.sqrt(Math.pow(targetVelo.get(i-1),2)+2*maxPathAccel*distance); //add rate limiter?
            targetVelo.add(Math.min(oldTargetVelo,newTargetVelo);
        }
    }
    public double distanceFormula(Point one, Point two){
        return Math.sqrt(Math.pow(two.x-one.x,2)+Math.pow(two.y-one.y,2));
    }


}
