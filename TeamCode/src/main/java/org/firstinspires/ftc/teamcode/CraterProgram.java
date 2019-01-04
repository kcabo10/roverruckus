package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Crater Program", group="Beep")
public class CraterProgram extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryTensorFlowObjectDetection tensorFlow = new LibraryTensorFlowObjectDetection(robot, telemetry);

    String goldPosition = "";
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
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
        robot.marker.setPosition(0);
        robot.basket.setPower(0);


        /**
        Wait for start button.
         */

        waitForStart();


// landing our robot

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(-17000);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

        getMineralPosition();
//
        robot.lift.setPower(0);

        runtime.reset();

        robot.latch.setPower(-1);

        while (runtime.seconds() <1.15){

        }
        robot.latch.setPower(0);


        gridNavigation.setGridPosition(.5417, .5417, 45);

        printTelemetry(0);
        telemetry.update();

        //goldPosition = "RIGHT";
        printTelemetry(10);
        telemetry.update();

        gridNavigation.driveToPosition(.75, .75, .2);

        int X = 0;
        int Y = 1;

       /**
        Change values to grab mineral
        */

        double[] RED_CRATER_LEFT = {.9, 1.8};
        double[] RED_CRATER_RIGHT = {1.8, .9};
        double[] RED_CRATER_CENTER = {1.35, 1.35};


        double[] RED_CRATER_MARKER = {-1.5, 2.5};
        double[] RED_CRATER_PARKING = {.4, 2.5};

        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_CRATER_LEFT[X], RED_CRATER_LEFT[Y], .8);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_LEFT[Y]);
                    printTelemetry(20);
//                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.lift.setTargetPosition(17000);
//                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.lift.setPower(1);
                    gridNavigation.driveToPositionNonBlocking(.35, 2.4, .6);
//                    runtime.reset();
//                    robot.latch.setPower(1);
//                    while (runtime.seconds() < 1.15) {
//                    }
//                    robot.latch.setPower(0);
//                    while (robot.rightFront.isBusy()) {
//                    }
                    gridNavigation.driveToPosition(RED_CRATER_MARKER[X], RED_CRATER_MARKER[Y], .7);
//                    sleep(400);
                    robot.marker.setPosition(90);
//                    sleep(400);
                    robot.marker.setPosition(0);
//                    sleep(400);
                    gridNavigation.driveToPositionBackwards(RED_CRATER_PARKING[X], RED_CRATER_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    runtime.reset();
                    robot.basket.setPower(-1);
                    while (runtime.seconds() < .7) {
                    }
                    robot.basket.setPower(0);
                    robot.armExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.armExtrusion.setTargetPosition(-5696);
                    armExtrusionPos = robot.armExtrusion.getCurrentPosition();
                    robot.armExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (armExtrusionPos < 5696) {
                        robot.armExtrusion.setPower(1);
                    }
                    robot.armExtrusion.setPower(0);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }
            break;

            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);

                if (goldPosition == "RIGHT") {
                    gridNavigation.driveToPosition(RED_CRATER_RIGHT[X], RED_CRATER_RIGHT[Y], .8);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_RIGHT[Y]);
                    printTelemetry(40);
//                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.lift.setTargetPosition(17000);
//                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.lift.setPower(1);
                    gridNavigation.driveToPositionBackwards(.9, .9, .6);
//                    runtime.reset();
//                    robot.latch.setPower(1);
//                    while (runtime.seconds() < 1.15) {
//                    }
//                    robot.latch.setPower(0);
//                    while (robot.rightFront.isBusy()) {
//                    }
                    gridNavigation.driveToPosition(.1, 2.3, .7);
                    //gridNavigation.driveToPosition(-1,2.5, .4);
                    gridNavigation.driveToPosition(RED_CRATER_MARKER[X], RED_CRATER_MARKER[Y], .7);
//                    sleep(400);
                    robot.marker.setPosition(90);
//                    sleep(400);
                    robot.marker.setPosition(0);
//                    sleep(400);
                    gridNavigation.driveToPositionBackwards(RED_CRATER_PARKING[X], RED_CRATER_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    runtime.reset();
                    robot.basket.setPower(-1);
                    while (runtime.seconds() < .7) {
                    }
                    robot.basket.setPower(0);
                    robot.armExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.armExtrusion.setTargetPosition(-5696);
                    armExtrusionPos = robot.armExtrusion.getCurrentPosition();
                    robot.armExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (armExtrusionPos < 5696) {
                    robot.armExtrusion.setPower(1);
                    }
                    robot.armExtrusion.setPower(0);
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }
            break;

            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                printTelemetry(60);

                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(RED_CRATER_CENTER[X], RED_CRATER_CENTER[Y], .8);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_CENTER[Y]);
                    printTelemetry(60);
//                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.lift.setTargetPosition(17000);
//                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.lift.setPower(1);
                    gridNavigation.driveToPositionBackwards(1,1,.6);
//                    runtime.reset();
//                    robot.latch.setPower(1);
//                    while (runtime.seconds() <1.15){
//                    }
//                    robot.latch.setPower(0);
//                    while (robot.rightFront.isBusy()) {
//                    }
                    gridNavigation.driveToPosition(.35,2.4,.7);
                    gridNavigation.driveToPosition(-1.3, 2.75,.7);
//                    sleep(400);
                    robot.marker.setPosition(90);
//                    sleep(400);
                    robot.marker.setPosition(0);
//                    sleep(400);
                    gridNavigation.driveToPositionBackwards(RED_CRATER_PARKING[X],RED_CRATER_PARKING[Y],.7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    runtime.reset();
                    robot.basket.setPower(-1);
                    while (runtime.seconds() <.7){
                    }
                    robot.basket.setPower(0);
                    robot.armExtrusion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.armExtrusion.setTargetPosition(-5696);
                    armExtrusionPos = robot.armExtrusion.getCurrentPosition();
                    robot.armExtrusion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (armExtrusionPos < 5696) {
                        robot.armExtrusion.setPower(1);
                    }
                    robot.armExtrusion.setPower(0);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(70);
                }

                telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_LEFT[X]);
                telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_LEFT[Y]);

                telemetry.update();
                break;

            default:
                telemetry.addData("Telemetry", "Didn't see gold pos");
                telemetry.update();
            break;

        }
        stop();

    }


    private void printTelemetry(int codePos) {
        telemetry.addData("Gold Pos", goldPosition);
        telemetry.addData("Code Position", codePos);
        telemetry.addData("Turn Angle", gridNavigation.turnAngle);
        telemetry.addData("Starting Angle", gridNavigation.StartingAngle);
        telemetry.update();
    }


    public void getMineralPosition() {
        int debounceCount = 0;
        long startTime = 0;
        String previousPosition;



        //goldPosition = "RIGHT";
        //previousPosition = "UNKNOWN";

//        goldPosition = "RIGHT";


        goldPosition = tensorFlow.findMineral();


//        telemetry.addData("MAP", "Running Sampling Order loop");
//        goldPosition = detector.getLastOrder().toString();

//        while (goldPosition == "UNKNOWN") {
//
//            // waits to turn after it has tried to check for the mineral pos
//
//            if (startTime != 0) gyroTurn.turnGyro(-5);
//
//            sleep(1000);
//
//            startTime = System.currentTimeMillis();
//
//
//            while (System.currentTimeMillis() < (startTime + 3000)) {
//
//                goldPosition = detector.getLastOrder().toString();
//
//                if (goldPosition == previousPosition) {
//                }
//                else {
//                    previousPosition = goldPosition;
//                    startTime = System.currentTimeMillis();
//
//                }
//                telemetry.addData("StartTime: ", startTime);
//                telemetry.addData("CurrentTime: ", System.currentTimeMillis());
//                telemetry.addData("Prev Position:  ", previousPosition);
//                telemetry.addData("Gold Position:  ", goldPosition);
//                telemetry.update();
//            }

//
//            telemetry.addData("Current Order", detector.getCurrentOrder().toString()); // The current result for the frame
//            telemetry.addData("Last Order", detector.getLastOrder().toString()); // The last known result


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
//        detector.disable();
}
//}