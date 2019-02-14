package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.SensorMB1242;


/**
 * Created by vasudevfamily on 8/31/17.
 *
 * This library contains the grid navigation program, which utilizes a virtual grid
 * with origin (0, 0) starting the center of the field.  The angle fo 0 degrees
 * begins on the positive X axis and moves counterclockwise
 */

public class LibraryGridNavigation {

    HardwareBeep robot;// = new HardwareBeep();
    //LibraryGyro gyro;// = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
//    SensorMB1242 leftUS = robot.leftSonic;
//    SensorMB1242 rightUS = robot.rightSonic;
    private ElapsedTime runtime = new ElapsedTime();
    Telemetry telemetry;
    int i = 0;

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
    public void setGridPosition(double xPosition, double yPosition, float angle) {
        xOrigin = xPosition;
        yOrigin = yPosition;
        StartingAngle = angle;
        System.out.println("setGridPos to (" + xPosition + ", " + yPosition + ") angle " + angle);
    }

//    //The angle 0 degrees starts on the positive X axis and moves counterclockwise
//    public double[] getGridPosition() {
//        double myPos[];
//        myPos[0] = xOrigin;
//        myPos[1] = yOrigin;
//        return myPos;
//    }

    public float getTurnAngleValuesOnly(double xDestination, double yDestination) {

        float tanAngle = 0; // comment

        double xLeg = xDestination - xOrigin;

        double yLeg = yDestination - yOrigin;

        double theta = Math.atan2(yLeg, xLeg);
        //atan2 automatically corrects for the limited domain of the inverse tangent function
        System.out.println("xLeg is " + xLeg);
        System.out.println("yLeg is " + yLeg);
        tanAngle = (float) Math.toDegrees(theta);
        if(tanAngle > 180){
            tanAngle = tanAngle - 360;
        }
        else if(tanAngle < -180){
            tanAngle = tanAngle + 360;
        }
        System.out.println("Start Angle is " + StartingAngle);
        System.out.println("Tangent Angle is " + tanAngle);
        xOrigin = xDestination;
        yOrigin = yDestination;

        turnAngle = tanAngle - StartingAngle;
//            if(180 > turnAngle){
//                turnAngle = turnAngle - 360;
//            }
//            else if(turnAngle < -180){
//                turnAngle = turnAngle + 360;
//            }
//            else{
//
//            }
        System.out.println("Turn angle " + turnAngle);
        StartingAngle = tanAngle;

        return turnAngle;

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
        while((tanAngle > 180) || (tanAngle < -180)) {
            if (tanAngle > 180) {
                tanAngle = tanAngle - 360;
            } else if (tanAngle < -180) {
                tanAngle = tanAngle + 360;
            }
        }

        System.out.println("Start Angle is " + StartingAngle);
        telemetry.addData("Start Angle is ", StartingAngle);
        System.out.println("Tangent Angle is " + tanAngle);
        telemetry.addData("Tangent Angle is ", tanAngle);
        xOrigin = xDestination;
        yOrigin = yDestination;

        turnAngle = tanAngle - StartingAngle;
//            if(180 > turnAngle){
//                turnAngle = turnAngle - 360;
//            }
//            else if(turnAngle < -180){
//                turnAngle = turnAngle + 360;
//            }
//            else{
//
//            }
        System.out.println("Turn angle " + turnAngle);
        StartingAngle = tanAngle;
        telemetry.update();

        return turnAngle;

    }

    public double getDriveDistance(double xDestination, double yDestination) {

        double xLeg = xDestination - xOrigin;

        double yLeg = yDestination - yOrigin;

        Distance = ((Math.hypot(xLeg, yLeg) * 24) / 12.57) * 537.6 * GEAR_RATIO_SCALING_FACTOR;
        /** The input for each grid coordiante is one tile, so first we multiply the input
         * by size of one tile, which is 24 inches.  Then we divide that value by the distance
         * covered by one rotation of our wheels, which is 12.57 inches.  We then multiply that
         * value by the number of encoder ticks per rotation of the motors we are using.  Finally,
         * we multiply that encoder value by the gear ratio that is set up for the wheel assembly we use.
*/
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
    public void driveToPositionNonBlocking(double xDestination, double yDestination, double power) {

        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);

        gyroDrive.gyro.turnGyro(turnAngle);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFront.setTargetPosition((int) (Distance));
        robot.leftBack.setTargetPosition((int) (Distance));
        robot.rightFront.setTargetPosition((int) (Distance));
        robot.rightBack.setTargetPosition((int) (Distance));

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(power);

    }

    //The grid is set such as that the origin (0, 0) is at the center and each grid point is 2 feet from the next point
    public void driveToPositionValuesOnly(double xDestination, double yDestination, double power) {

        System.out.println("%n%ndriveToPosValuesOnly to (" + xDestination + ", " + yDestination + ")");

        getDriveDistance(xDestination, yDestination);
        getTurnAngleValuesOnly(xDestination, yDestination);
    }

    //The grid is set such as that the origin (0, 0) is at the center and each grid point is 2 feet from the next point
    public void driveToPosition(double xDestination, double yDestination, double power) {

        double PCoeff = .01;

        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);
//
//        telemetry.addData("driveToPos: distance", Distance);
//        telemetry.addData("driveToPos: power", power);
//        telemetry.addData("driveToPos: PCoeff", PCoeff);
//        telemetry.addData("driveToPos: turnAngle", turnAngle);
//        telemetry.update();
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        while (timer.seconds()<4) {}


        gyroDrive.gyro.turnGyro(turnAngle);

        //gyroDrive.gyroDrive(power, (int) Distance, 0.0);


//        telemetry.addData("driveToPos: turnAngle", turnAngle);
//        telemetry.addData("driveToPos: getAngle", gyroDrive.gyro.getAngle());
//        telemetry.update();
//        timer.reset();
//        while (timer.seconds()<4) {}

        gyroDrive.gyroDriveVariableP(power, (int) Distance, turnAngle, PCoeff);

    }
//
//    public void driveToPositionSonic(double xDestination, double yDestination, double power) {
//
//        getDriveDistance(xDestination, yDestination);
//
//        rearUS.startAutoPing(40);
//            if (runtime.milliseconds() > 200){
//
//                telemetry.addData("Distance",rearUS.getDistance());
//                telemetry.addData("Incrementor", i++);
//                telemetry.update();
//                rearUS.ping();
//                runtime.reset();
//            }
//            if (rearUS.getDistance() == 40){
//                gyroDrive.gyroDrive(power, (int) Distance, 0.0);
//            }
//            if (rearUS.getDistance() > 40){
//                gyro.turnGyro(20);
//                gyroDrive.gyroDrive(power, (int) Distance, 0.0);
//            }
//            if (rearUS.getDistance() < 40){
//
//            }

//    }

    public void init(HardwareBeep myRobot, LibraryGyro myGyro, Telemetry myTelemetry) {
        robot = myRobot;
        //gyro = myGyro;
        telemetry = myTelemetry;
        gyroDrive.init(robot, telemetry, robot.leftFront);
    }

    public void driveToPositionBackwards(double xDestination, double yDestination, double power) {
        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);

//        turnAngle = (turnAngle - 180);

        if(turnAngle > 180){
            turnAngle = turnAngle - 180;
            StartingAngle = (StartingAngle - 180);

        }
        else if(turnAngle < -180){
            turnAngle = turnAngle + 180;
            StartingAngle = (StartingAngle + 180);

        }
        else {
        }

        gyroDrive.gyro.turnGyro(turnAngle);
        gyroDrive.gyroDrive(-power, -(int) Distance, 0.0);

    }

    public void driveToPositionBackwardsValuesOnly(double xDestination, double yDestination, double power) {
        getDriveDistance(xDestination, yDestination);
        getTurnAngleValuesOnly(xDestination, yDestination);

        turnAngle = (turnAngle - 180);
        StartingAngle = (StartingAngle - 180);


        System.out.println("driveToPositionBackwardsValueOnly to with turn angle " + turnAngle + " and Starting Angle " + StartingAngle);

    }

    public void driveToPositionReverse(double xDestination, double yDestination, double power) {
        getDriveDistance(xDestination, yDestination);
        getTurnAngle(xDestination, yDestination);

        turnAngle = (turnAngle + 180);
        StartingAngle = (StartingAngle + 180);

        gyroDrive.gyro.turnGyro(turnAngle);
        gyroDrive.gyroDrive(-power, -(int) Distance, 0.0);

    }

    public void driveToPositionReverseValuesOnly(double xDestination, double yDestination, double power) {
        getDriveDistance(xDestination, yDestination);
        getTurnAngleValuesOnly(xDestination, yDestination);

        turnAngle = (turnAngle + 180);
        StartingAngle = (StartingAngle + 180);
        System.out.println("driveToPositionBackwardsValueOnly to with turn angle " + turnAngle + " and Starting Angle " + StartingAngle);

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