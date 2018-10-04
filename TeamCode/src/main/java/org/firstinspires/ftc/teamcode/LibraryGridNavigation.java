package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by vasudevfamily on 8/31/17.
 */

public class LibraryGridNavigation {

    LibraryGyro gyro = new LibraryGyro();


    double xOrigin;
    //X1 is starting X coordinate
    double xDestination;
    //X2 is X destination
    double yOrigin;
    //Y1 is starting Y coordinate
    double yDestination;
    //Y2 is Y destination
    float StartingAngle;
    //X1 is starting X coordinate
    //Y1 is starting Y coordinate
    double Distance;
    float turnAngle = 0f;

    //The angle 0 degrees starts on the positive X axis and moves counterclockwise
    public void setGridPosition(double xPosition, double yPosition, float angle){
        xOrigin = xPosition;
        yOrigin = yPosition;
        StartingAngle = angle;
    }

    public float getTurnAngle(double xDestination, double yDestination) {

        float tanAngle = 0; // comment

        double xLeg = xDestination - xOrigin;

        double yLeg = yDestination - yOrigin;

//        Distance = ((Math.hypot(xLeg, yLeg)*12)/12.57)*1120;
        // Distance is in encoder ticks

        double theta = Math.atan2(yLeg, xLeg);
        //atan2 automatically corrects for the limited domain of the inverse tangent function
        System.out.println(xLeg);
        System.out.println(yLeg);
        tanAngle = (float) Math.toDegrees(theta);
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

    public double getDriveDistance(double xDestination, double yDestination){

        double xLeg = xDestination - xOrigin;

        double yLeg = yDestination - yOrigin;

        Distance = ((Math.hypot(xLeg, yLeg)*12)/12.57)*1120;
        // Distance is in encoder ticks

        System.out.println("Hypotenus is " + Distance);


        return Distance;
    }

    //The grid is set such as that the origin (0, 0) is at the center and each grid point is 2 feet from the next point
    public void driveToPosition(double xDestination, double yDestination, double power){

        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);

        gyro.turnGyro(turnAngle);

        double COUNTS_PER_INCH = 1120;
        gyro.LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gyro.LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro.LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro.RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro.RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gyro.LeftFront.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        gyro.LeftBack.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        gyro.RightFront.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        gyro.RightBack.setTargetPosition((int)(Distance * COUNTS_PER_INCH));

        gyro.LeftFront.setPower(power);
        gyro.LeftBack.setPower(power);
        gyro.RightFront.setPower(power);
        gyro.RightBack.setPower(power);

        while((gyro.LeftFront.getCurrentPosition() < (Distance * COUNTS_PER_INCH))
                && (gyro.RightFront.getCurrentPosition() < (Distance * COUNTS_PER_INCH))){}

        gyro.LeftFront.setPower(0);
        gyro.LeftBack.setPower(0);
        gyro.RightFront.setPower(0);
        gyro.RightBack.setPower(0);

        gyro.LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void init(){

    }

}


/*
    public void angleCorrection(double L1, double L2, double tanAngle) {
        double getTurnAngle = 0;
        if((L1 < 0) && (L2 < 0)){
            getTurnAngle = tanAngle - StartingAngle + 180;
        }
        else if(L1 < 0){
            getTurnAngle = tanAngle - StartingAngle + 180;
        }
        else{
            getTurnAngle = tanAngle - StartingAngle;
        }
        System.out.println(getTurnAngle);
        StartingAngle = getTurnAngle;
        }
}
*/