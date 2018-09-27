package org.firstinspires.ftc.teamcode;


/**
 * Created by vasudevfamily on 8/31/17.
 */

public class GridNavigationImplimentation {

    double xOrigin = 0;
    //X1 is starting X coordinate
    double xDestination;
    //X2 is X destination
    double yOrigin= 0;
    //Y1 is starting Y coordinate
    double yDestination;
    //Y2 is Y destination
    double StartingAngle = 0;
    //X1 is starting X coordinate
    //Y1 is starting Y coordinate
    double Distance;
    double turnAngle = 0;


    GridNavigationImplimentation impl = new GridNavigationImplimentation();

    public static void main(String args[]) {

        GridNavigationImplimentation impl = new GridNavigationImplimentation();

        impl.GridNavTurnAngle(2,3);
        //  String returnValue = impl.add2(startValue);
        // System.out.println("return value in main: " + returnValue);
    }

    public void SetGridPosition(double xStart, double yStart, double StartAngle){
        xOrigin = xStart;
        yOrigin = yStart;
        StartingAngle = StartAngle;
    }

    public double GridNavTurnAngle(double xDestination, double yDestination) {

        double tanAngle = 0; // comment

        double xLeg = xDestination - xOrigin;

        double yLeg = yDestination - yOrigin;

//        Distance = ((Math.hypot(xLeg, yLeg)*12)/12.57)*1120;
        // Distance is in encoder ticks

        double theta = Math.atan2(yLeg, xLeg);
        //atan2 automatically corrects for the limited domain of the inverse tangent function
        System.out.println(xLeg);
        System.out.println(yLeg);
        tanAngle = Math.toDegrees(theta);
        System.out.println("Start Angle is " + StartingAngle);
        System.out.println("Tangent Angle is " + tanAngle);
//        System.out.println("Hypotenus is " + Distance);
        xOrigin = xDestination;
        yOrigin = yDestination;

        turnAngle = tanAngle - StartingAngle;
        System.out.println("Turn angle " + turnAngle);
        StartingAngle = tanAngle;

//        impl.GridNav(2,3,1.0);
//        impl.GridNav(4,3,1.0);
        return turnAngle;

    }

    public double GridNavDriveDistance(double xDestination, double yDestination){

        double xLeg = xDestination - xOrigin;

        double yLeg = yDestination - yOrigin;

        Distance = ((Math.hypot(xLeg, yLeg)*12)/12.57)*1120;
        // Distance is in encoder ticks

        System.out.println("Hypotenus is " + Distance);


        return Distance;
    }

}


/*
    public void angleCorrection(double L1, double L2, double tanAngle) {
        double turnAngle = 0;
        if((L1 < 0) && (L2 < 0)){
            turnAngle = tanAngle - StartingAngle + 180;
        }
        else if(L1 < 0){
            turnAngle = tanAngle - StartingAngle + 180;
        }
        else{
            turnAngle = tanAngle - StartingAngle;
        }
        System.out.println(turnAngle);
        StartingAngle = turnAngle;
        }
}
*/