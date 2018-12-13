package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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


        /**
        Wait for start button.
         */



        waitForStart();


        // landing our robot

//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setTargetPosition(17000);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(1);
//
//        while (opModeIsActive() &&
//                robot.lift.isBusy()){
//            getMineralPosition();
//            telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
//            telemetry.update();
////        }
//
//        robot.lift.setPower(0);
//
//        runtime.reset();
//
//        robot.latch.setPower(-1);
//
//        while (runtime.seconds() <1.15){
//
//        }
//        robot.latch.setPower(-.1);

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(17000);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

        while (opModeIsActive() &&
                robot.lift.isBusy()){
            getMineralPosition();
            telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
            telemetry.update();
        }

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

        double[] RED_CRATER_LEFT = {.65, 1.6};
        double[] RED_CRATER_RIGHT = {1.6, .65};
        double[] RED_CRATER_CENTER = {1.35, 1.35};


        double[] RED_CRATER_MARKER = {-1.8, 2};
        double[] RED_CRATER_PARKING = {.8, 2};



        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_CRATER_LEFT[X], RED_CRATER_LEFT[Y], .4);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_LEFT[Y]);
                    sleep(2000);
                    gridNavigation.driveToPositionBackwards(.65,1.5,.4);
                    gridNavigation.driveToPosition(.35,2,.4);
                    gridNavigation.driveToPosition(RED_CRATER_MARKER[X],RED_CRATER_MARKER[Y],.4);
                    gridNavigation.driveToPosition(RED_CRATER_PARKING[X],RED_CRATER_PARKING[Y],.4);

                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }



            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);

                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(RED_CRATER_RIGHT[X], RED_CRATER_RIGHT[Y], .4);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_RIGHT[Y]);
                    sleep(2000);
                    gridNavigation.driveToPositionBackwards(.8,.8,.4);
                    sleep(2000);
                    gridNavigation.driveToPosition(.35,2,.4);
                    sleep(2000);
                    gridNavigation.driveToPosition(RED_CRATER_MARKER[X],RED_CRATER_MARKER[Y],.4);
                    sleep(2000);
                    gridNavigation.driveToPosition(RED_CRATER_PARKING[X],RED_CRATER_PARKING[Y],.4);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }



            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                printTelemetry(60);

                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(RED_CRATER_CENTER[X], RED_CRATER_CENTER[Y], .4);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_CENTER[Y]);
                    sleep(3000);
                    gridNavigation.driveToPosition(1,1,.4);
                    gridNavigation.driveToPosition(0,2,.4);
                    gridNavigation.driveToPosition(RED_CRATER_MARKER[X],RED_CRATER_MARKER[Y],.4);
                    gridNavigation.driveToPosition(RED_CRATER_PARKING[X],RED_CRATER_PARKING[Y],.4);
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

        }
        //foundTargetName, mineral postion,
        //code pos

        //telemetry.update();


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