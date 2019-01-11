package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Depot Program No Light", group="Beep")
public class DepotProgramNoLight extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryDogeforia dogeforia = new LibraryDogeforia(robot, telemetry);
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryTensorFlowObjectDetectionNoLight tensorFlow = new LibraryTensorFlowObjectDetectionNoLight(robot, telemetry);

    String goldPosition = "";
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    int armExtrusionPos, liftPos;


    /**
    Called when init button is  pressed.
    */

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        robot.init(hardwareMap);
        gridNavigation.init(robot, gyroTurn, telemetry);
        gyroTurn.init(robot, telemetry);
        gyroDrive.init(robot, telemetry, robot.rightBack);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        robot.latch.setPower(0);
        robot.marker.setPosition(0);
        robot.basket.setPower(0);

        /**
        Wait for start button.
         */
        waitForStart();

        // landing our robot

//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setTargetPosition(-17000);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(1);

        gridNavigation.setGridPosition(.8281, .8281, 45);
        printTelemetry(10);
        telemetry.update();
        gridNavigation.driveToPosition(.9, .9, .8);

            getMineralPosition();



//        robot.lift.setPower(0);
//        runtime.reset();
//        robot.latch.setPower(-1);
//
//        while (runtime.seconds() <1.15){
//
//        }
//        robot.latch.setPower(0);
//        robot.lift.setPower(0);
//        runtime.reset();
//
        gridNavigation.setGridPosition(.8281, .8281, 45);
        printTelemetry(10);
        telemetry.update();
        gridNavigation.driveToPosition(1, 1, .8);

        int X = 0;
        int Y = 1;

       /**
        Change values to grab mineral
        */

        double[] RED_DEPOT_LEFT = {1.3, 2.6};
        double[] RED_DEPOT_RIGHT = {2.6, 1.3};
        double[] RED_DEPOT_CENTER = {1.5, 1.5};

        double[] LEFT_RED_DEPOT_MARKER = {2.3,2.7};
        double[] RIGHT_RED_DEPOT_MARKER = {2.6,2.3};
        double[] CENTER_RED_DEPOT_MARKER = {2.2,2.2};

        double[] RED_DEPOT_PARKING = {2.6, -.5};

        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);
                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .7);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);

                    gridNavigation.driveToPositionBackwards(1, 2, .7);
                    gridNavigation.driveToPosition(LEFT_RED_DEPOT_MARKER [X], LEFT_RED_DEPOT_MARKER [Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);

                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);

                    gridNavigation.driveToPosition(2.8, 1.6, .7);
                    gridNavigation.setGridPosition(2.6,1.6,-75);
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }

                break;

            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);
                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .7);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
                    gridNavigation.driveToPosition(RIGHT_RED_DEPOT_MARKER [X], RIGHT_RED_DEPOT_MARKER [Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);

                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }

                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(2.6, 1.6, .7);
                    gridNavigation.setGridPosition(2.6,1.6,-90 );
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }

                break;

            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                printTelemetry(60);
                if (goldPosition == "CENTER"){
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_CENTER[Y]);
//                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.lift.setTargetPosition(-17000);
//                    liftPos = robot.lift.getCurrentPosition();
//                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    while (liftPos < -17000) {
//                    robot.lift.setPower(1);
//                    }
                    gridNavigation.driveToPosition(CENTER_RED_DEPOT_MARKER[X], CENTER_RED_DEPOT_MARKER[Y],.7);
//                    runtime.reset();
//                    robot.latch.setPower(1);
//                    while (runtime.seconds() < 1.15) {
//                    }
//                    robot.latch.setPower(0);
//                    while (robot.rightFront.isBusy()) {
//                    }
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);

                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }

                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(2.7, 1.6, .7);
                    gridNavigation.setGridPosition(2.6,1.6,-60);
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(70);
                }

                telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);

                telemetry.update();
                break;


            default:
                telemetry.addData("Telemetry", "Didn't see gold pos");
                telemetry.update();
                break;

        }
    }


    private void printTelemetry(int codePos) {
        telemetry.addData("Gold Pos", goldPosition);
        telemetry.addData("Code Position", codePos);
        telemetry.update();
    }


    public void getMineralPosition() {
        int debounceCount = 0;
        long startTime = 0;
        String previousPosition;
        goldPosition = tensorFlow.findMineral();

            switch (goldPosition) {
                case ("LEFT"):
                    telemetry.addData("Telemetry", "Left Position");
                    telemetry.update();
                    break;
                case ("RIGHT"):
                    telemetry.addData("Telemetry", "Right Position");
                    telemetry.update();
                    break;
                case ("CENTER"):
                    telemetry.addData("Telemetry", "Center Position");
                    telemetry.update();
                    break;
                case ("UNKNOWN"):
                    telemetry.addData("Telemetry", "Unknown Position");
                    telemetry.update();
                    break;
            }
            telemetry.update();
        }
    }