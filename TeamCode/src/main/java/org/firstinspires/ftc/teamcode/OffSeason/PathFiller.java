package org.firstinspires.ftc.teamcode.OffSeason;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;
public class PathFiller {

    public static ArrayList<Point> filler(ArrayList<Point> skeletonList, double spacing){
        int originalSize = skeletonList.size();
        ArrayList<Point> filler = new ArrayList<Point>();
        for(int i = 0; i<originalSize-2; i++){
            Vector2d tempVector = new Vector2d(skeletonList.get(i+1).x-skeletonList.get(i).x,skeletonList.get(i+1).y-skeletonList.get(i).y);
            int pointsThatFit = (int)Math.ceil(tempVector.magnitude()/spacing);
            tempVector = scaling*tempVector.div(tempVector.magnitude());
            for(int n = 0; n<pointsThatFit; n++){
                filler.add(skeletonList.get(i).plus(tempVector.times(n)));
                tempVector = tempVector.div(n);
            }
        }
        filler.add(filler.get(filler.size()-1));
        return filler;
    }
    public static ArrayList<Point> smoother(ArrayList<Point> sharp, double a, double b, double tolerance){

    }
}
