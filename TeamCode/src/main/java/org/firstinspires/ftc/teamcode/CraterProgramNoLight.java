package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Crater Program No Light", group = "Beep")
public class CraterProgramNoLight extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryTensorFlowObjectDetectionNoLight tensorFlow = new LibraryTensorFlowObjectDetectionNoLight(robot, telemetry);
    String goldPosition = "";
    int armExtrusionPos, liftPos;

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


        /**
         Wait for start button.
         */

        waitForStart();


// landing our robot

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(-18000);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

        getMineralPosition();

        robot.lift.setPower(0);
        runtime.reset();
        robot.latch.setPower(-1);

        while (runtime.seconds() < 1.15) {

        }
        robot.latch.setPower(0);
        robot.lift.setPower(0);
        runtime.reset();

        gridNavigation.setGridPosition(.6076, .6076, 45);

        int X = 0;
        int Y = 1;

        /**
         Change values to grab mineral
         */

        double[] RED_CRATER_LEFT = {1, 1.2916}; //.9322
        double[] RED_CRATER_RIGHT = {1.3125, .8958};
        double[] RED_CRATER_CENTER = {1.1, 1.1};

        double[] RED_CRATER_MARKER = {-1.5, 2.8};
        double[] RIGHT_CRATER_MARKER = {-1.5, 2.5};
        double[] LEFT_CRATER_MARKER = {-1.5, 2.9};

        double[] RED_CRATER_PARKING = {.8, 2.7};

        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(.8, .8, .5);
                    lowerLift();
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(750);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(RED_CRATER_LEFT[X], RED_CRATER_LEFT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_LEFT[Y]);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    gridNavigation.driveToPosition(0, 2.5, .5);
                    gridNavigation.driveToPosition(LEFT_CRATER_MARKER[X], LEFT_CRATER_MARKER[Y], .5);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(450);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    robot.intake.setPower(.6);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-450);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    gridNavigation.driveToPosition(RED_CRATER_PARKING[X], RED_CRATER_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }
                break;

            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);

                if (goldPosition == "RIGHT") {
                    gridNavigation.driveToPosition(.8, .8, .5);
                    lowerLift();
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(750);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(RED_CRATER_RIGHT[X], RED_CRATER_RIGHT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_RIGHT[Y]);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    gridNavigation.driveToPosition(0, 2.5, .5);
                    gridNavigation.driveToPosition(RIGHT_CRATER_MARKER[X], RIGHT_CRATER_MARKER[Y], .5);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(450);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-450);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    gridNavigation.driveToPosition(RED_CRATER_PARKING[X], RED_CRATER_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }
                break;

            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                if (goldPosition == "CENTER") {
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(750);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(RED_CRATER_CENTER[X], RED_CRATER_CENTER[Y], .5);
                    lowerLift();
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    gridNavigation.driveToPosition(0, 2.5, .5);
                    gridNavigation.driveToPosition(RED_CRATER_MARKER[X], RED_CRATER_MARKER[Y], .5);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(450);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-450);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    gridNavigation.driveToPosition(RED_CRATER_PARKING[X], RED_CRATER_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }
                break;

            default:
                telemetry.addData("Telemetry", "Didn't see gold pos");
                telemetry.update();
                break;
        }
        telemetry.addData("Parked Ready to pull out arm", "");
        telemetry.update();

        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setTargetPosition(-537);
        robot.leftBack.setTargetPosition(-537);
        robot.rightFront.setTargetPosition(-537);
        robot.rightBack.setTargetPosition(-537);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setPower(.5);
        robot.leftFront.setPower(.5);
        robot.rightBack.setPower(.5);
        robot.rightFront.setPower(.5);

        while (robot.rightFront.isBusy()) {

        }
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(-400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);
        while (robot.arm.isBusy()) {
        }
        robot.arm.setPower(.111);


        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setTargetPosition(500);
        robot.leftBack.setTargetPosition(500);
        robot.rightFront.setTargetPosition(500);
        robot.rightBack.setTargetPosition(500);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setPower(.5);
        robot.leftFront.setPower(.5);
        robot.rightBack.setPower(.5);
        robot.rightFront.setPower(.5);

        while (robot.rightFront.isBusy()) {

        }
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);
        while (robot.arm.isBusy()) {
        }
        robot.arm.setPower(.111);

        robot.intake.setPower(1);
        sleep(1000);
        robot.intake.setPower(0);


        while (true) {

            //start intake
            robot.intake.setPower(1);

            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setTargetPosition(-250);
            robot.leftBack.setTargetPosition(-250);
            robot.rightFront.setTargetPosition(-250);
            robot.rightBack.setTargetPosition(-250);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setPower(.5);
            robot.leftFront.setPower(.5);
            robot.rightBack.setPower(.5);
            robot.rightFront.setPower(.5);

            while (robot.rightFront.isBusy()) {

            }
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);

            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setTargetPosition(250);
            robot.leftBack.setTargetPosition(250);
            robot.rightFront.setTargetPosition(250);
            robot.rightBack.setTargetPosition(250);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setPower(.5);
            robot.leftFront.setPower(.5);
            robot.rightBack.setPower(.5);
            robot.rightFront.setPower(.5);

            while (robot.rightFront.isBusy()) {

            }
            robot.leftBack.setPower(0);
            robot.leftFront.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);

            robot.intake.setPower(0);
            /**stop and reset encoders on motors
             * set target position forward
             * set power 1
             * while is busy do nothing
             * set power 0 to all motors
             *
             * stop and reset encoders on motors
             * set target pos backward
             * set power 1
             * while is busy do nothing
             * set power 0 to all motors
             *
             */
        }
    }

    private void printTelemetry(int codePos) {
        telemetry.addData("Gold Pos", goldPosition);
        telemetry.addData("Code Position", codePos);
        telemetry.addData("Turn Angle", gridNavigation.turnAngle);
        telemetry.addData("Starting Angle", gridNavigation.StartingAngle);
        telemetry.update();
    }

    private void lowerLift() {
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(14500);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

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
            default:
                telemetry.addData("Telemetry", "Unknown Position");
                telemetry.update();
                goldPosition = "CENTER";
                break;
        }

        telemetry.update();
    }
}