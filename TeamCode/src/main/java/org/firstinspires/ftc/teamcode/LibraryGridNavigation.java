package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by vasudevfamily on 8/31/17.
 *
 * This library contains the grid navigation program, which utilizes a virtual grid
 * with origin (0, 0) starting the center of the field.  The angle fo 0 degrees
 * begins on the positive X axis and moves counterclockwise
 */

public class LibraryGridNavigation {

    HardwareBeep robot;// = new HardwareBeep();
    LibraryGyro gyro;// = new LibraryGyro();


    double xOrigin = 0;
    //X1 is starting X coordinate
    double xDestination;
    //X2 is X destination
    double yOrigin = 0;
    //Y1 is starting Y coordinate
    double yDestination;
    //Y2 is Y destination
    float StartingAngle = 0;
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
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        robot.leftBack.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        robot.rightFront.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        robot.rightBack.setTargetPosition((int)(Distance * COUNTS_PER_INCH));

        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(power);

        while((robot.leftFront.getCurrentPosition() < (Distance * COUNTS_PER_INCH))
                && (robot.rightFront.getCurrentPosition() < (Distance * COUNTS_PER_INCH))){}

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void init(HardwareBeep myRobot){
        robot = myRobot;
        gyro = new LibraryGyro(robot);
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