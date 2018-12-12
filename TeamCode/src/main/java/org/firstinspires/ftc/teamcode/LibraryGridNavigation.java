package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


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
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    private ElapsedTime runtime = new ElapsedTime();
    Telemetry telemetry;

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
    double GEAR_RATIO_SCALING_FACTOR = 1.2857142857;//(35/45);

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

        double theta = Math.atan2(yLeg, xLeg);
        //atan2 automatically corrects for the limited domain of the inverse tangent function
        System.out.println(xLeg);
        telemetry.addData("X pos", xLeg);
        System.out.println(yLeg);
        telemetry.addData("Y pos", yLeg);
        tanAngle = (float) Math.toDegrees(theta);
        if(tanAngle > 180){
            tanAngle = tanAngle - 360;
        }
        else if(tanAngle < -180){
            tanAngle = tanAngle + 360;
        }
        System.out.println("Start Angle is " + StartingAngle);
        telemetry.addData("Start Angle is ", StartingAngle);
        System.out.println("Tangent Angle is " + tanAngle);
        telemetry.addData("Tangent Angle is ", tanAngle);
        xOrigin = xDestination;
        yOrigin = yDestination;

        turnAngle = tanAngle - StartingAngle;
        System.out.println("Turn angle " + turnAngle);
        StartingAngle = tanAngle;
        telemetry.update();

        return turnAngle;

    }

    public double getDriveDistance(double xDestination, double yDestination){

        double xLeg = xDestination - xOrigin;

        double yLeg = yDestination - yOrigin;

        Distance = ((Math.hypot(xLeg, yLeg)*24)/12.57)*537.6 * GEAR_RATIO_SCALING_FACTOR;
        // Distance is in encoder ticks

        System.out.println("Drive Distance is " + Distance);

        /* START TEST CODE FOR SHOWING PRINTS
        telemetry.addData("Drive Distance is ", Distance);
        telemetry.addData("Gear Scale Factor", GEAR_RATIO_SCALING_FACTOR);
        telemetry.addData("xOrigin", xOrigin);
        telemetry.addData("yOrigin", yOrigin);
        telemetry.addData("xDestination", xDestination);
        telemetry.addData("yDestimation", yDestination);
        telemetry.addData("xLeg", xLeg);
        telemetry.addData("yLeg", yLeg);
        telemetry.update();

        runtime.reset();

        while((runtime.seconds() < 4)) {
        }
        /* END TEST CODE */
        return Distance;
    }

    //The grid is set such as that the origin (0, 0) is at the center and each grid point is 2 feet from the next point
    public void driveToPositionGyro(double xDestination, double yDestination, double power){
        // NEEDS ADDL TESTING
        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);

        gyro.turnGyro(turnAngle);

        gyroDrive.driveGyro(power, (int)(Distance));
    }

    //The grid is set such as that the origin (0, 0) is at the center and each grid point is 2 feet from the next point
    public void driveToPosition(double xDestination, double yDestination, double power){

        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);

        gyro.turnGyro(turnAngle);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFront.setTargetPosition((int)(Distance));
        robot.leftBack.setTargetPosition((int)(Distance));
        robot.rightFront.setTargetPosition((int)(Distance));
        robot.rightBack.setTargetPosition((int)(Distance));

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(power);

        runtime.reset();

        while((robot.rightFront.isBusy() && robot.leftFront.isBusy())){

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to ", Distance);
            telemetry.addData("Left side",  "Current position",
                    robot.leftFront.getCurrentPosition());
            telemetry.addData("Right Drive Current Position", robot.rightFront.getCurrentPosition());
            telemetry.addData("Speed", power);
            telemetry.update();
        }

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void init(HardwareBeep myRobot, LibraryGyro myGyro, Telemetry myTelemetry){
        robot = myRobot;
        gyro = myGyro;
        telemetry = myTelemetry;
        gyroDrive.init(robot, telemetry, robot.leftFront);
    }

    public void driveToPositionBackwards(double xDestination, double yDestination, double power){
        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);
        
        gyro.turnGyro(turnAngle - 180);

        StartingAngle = StartingAngle - 180;

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFront.setTargetPosition(-(int)(Distance));
        robot.leftBack.setTargetPosition(-(int)(Distance));
        robot.rightFront.setTargetPosition(-(int)(Distance));
        robot.rightBack.setTargetPosition(-(int)(Distance));

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightFront.setPower(-power);
        robot.rightBack.setPower(-power);

        runtime.reset();

        while((robot.rightFront.isBusy() && robot.leftFront.isBusy())){

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to ", Distance);
            telemetry.addData("Left side",  "Current position",
                    robot.leftFront.getCurrentPosition());
            telemetry.addData("Right Drive Current Position", robot.rightFront.getCurrentPosition());
            telemetry.addData("Speed", power);
            telemetry.update();
        }

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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