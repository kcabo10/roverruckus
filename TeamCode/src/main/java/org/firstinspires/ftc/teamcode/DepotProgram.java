package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Depot Program", group="Beep")
public class DepotProgram extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryDogeforia dogeforia = new LibraryDogeforia(robot, telemetry);
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryTensorFlowObjectDetection tensorFlow = new LibraryTensorFlowObjectDetection(robot, telemetry);

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


//        // landing our robot
//
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(-17000);
        //liftPos = robot.lift.getCurrentPosition();
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //while (liftPos < 17000) {
        robot.lift.setPower(1);
        //}

//        do {
//            telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
//            telemetry.update();
//        }
//        while (opModeIsActive() &&
//                robot.lift.isBusy());

        getMineralPosition();

        //robot.lift.setPower(0);

        runtime.reset();

        robot.latch.setPower(-1);

        while (runtime.seconds() <1.15){

        }
        robot.latch.setPower(0);

        gridNavigation.setGridPosition(.5417, .5417, 45);

        int codePos = 0;

        //goldPosition = "RIGHT";
        printTelemetry(10);
        telemetry.update();

        //telemetry.update();

        gridNavigation.driveToPosition(.75, .75, .4);

        int X = 0;
        int Y = 1;

       /**
        Change values to grab mineral
        */

        double[] RED_DEPOT_LEFT = {1, 2};
        double[] RED_DEPOT_RIGHT = {2, 1};
        double[] RED_DEPOT_CENTER = {1.5, 1.5};

        double[] LEFT_RED_DEPOT_MARKER = {1.8,2.5};
        double[] RIGHT_RED_DEPOT_MARKER = {2.6,1.8};
        double[] CENTER_RED_DEPOT_MARKER = {2,2};

        double[] RED_DEPOT_PARKING = {2.5, 0};

        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);
                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .4);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);

                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lift.setTargetPosition(15000);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lift.setPower(1);

                    runtime.reset();
                    robot.latch.setPower(1);
                    while (runtime.seconds() < 1.15) {
                    }
                    robot.latch.setPower(0);

                    gridNavigation.driveToPosition(1.8, 2.3, .2);

                    sleep(500);
                    runtime.reset();
                    robot.arm.setPower(-1);
                    while (runtime.seconds() <= .3) {
                    }
                    robot.arm.setPower(0);

                    runtime.reset();
                    robot.arm.setPower(1);
                    while (runtime.seconds() <= .4) {
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
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .4);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lift.setTargetPosition(15000);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lift.setPower(1);

                    runtime.reset();
                    robot.latch.setPower(1);
                    while (runtime.seconds() < 1.15) {
                    }
                    robot.latch.setPower(0);

                    gridNavigation.driveToPosition(2.3, 1.8, .2);

                    sleep(500);
                    runtime.reset();
                    robot.arm.setPower(-1);
                    while (runtime.seconds() <= .3) {
                    }
                    robot.arm.setPower(0);

                    runtime.reset();
                    robot.arm.setPower(1);
                    while (runtime.seconds() <= .4) {
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
                    gridNavigation.driveToPosition(RED_DEPOT_CENTER[X], RED_DEPOT_CENTER[Y], .4);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_CENTER[Y]);
                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lift.setTargetPosition(15000);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lift.setPower(1);

                    runtime.reset();
                    robot.latch.setPower(1);
                    while (runtime.seconds() < 1.15) {
                    }
                    robot.latch.setPower(0);

                    gridNavigation.driveToPosition(2, 2, .2);

                    sleep(500);
                    runtime.reset();
                    robot.arm.setPower(-1);
                    while (runtime.seconds() <= .3) {
                    }
                    robot.arm.setPower(0);

                    runtime.reset();
                    robot.arm.setPower(1);
                    while (runtime.seconds() <= .4) {
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