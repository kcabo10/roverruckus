package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Depot Program With Light", group="Beep")
public class DepotProgramWithLight extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryDogeforia dogeforia = new LibraryDogeforia(robot, telemetry);
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryTensorFlowObjectDetectionWithLight tensorFlow = new LibraryTensorFlowObjectDetectionWithLight(robot, telemetry);

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
//        robot.basket.setPosition(0);

        /**
         Wait for start button.
         */
        waitForStart();

        // landing our robot

//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setTargetPosition(-17500);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(1);

        getMineralPosition();

//        runtime.reset();

//        while ((goldPosition == "Unknown") && (runtime.seconds() < 5)){
//            gridNavigation.driveToPosition(.9,.9, .7);
//
//        }
//
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

        /**
         * took the hypotenuse line to the center of the robot is 20.625 inches
         */

        gridNavigation.setGridPosition(.6076, .6076, 45);
//        telemetry.addData("my Initial Grid Nav Pos", gridNavigation.getGridPosition());
//        telemetry.update();
//        sleep(2000);
//        gridNavigation.driveToPosition(1, 1, .8);

        int X = 0;
        int Y = 1;

        /**
         Change values to grab mineral
         */

        double[] RED_DEPOT_LEFT = {1.2, 2.5};
        double[] RED_DEPOT_RIGHT = {1.23, .875};
        double[] RED_DEPOT_CENTER = {1.1, 1.1};

        double[] RED_DEPOT_MARKER = {1.9,2.5};

        double[] RED_DEPOT_PARKING = {-.4, 2.6};

        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);
                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(.8,.8,.5);
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);
                    gridNavigation.driveToPosition(RED_DEPOT_MARKER [X], RED_DEPOT_MARKER [Y], .5);
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .5);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
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
                    gridNavigation.driveToPosition(.8,.8,.5);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-700);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(.5, 1.5, .5);
                    gridNavigation.driveToPosition(.5, 2.5, .5);
                    gridNavigation.driveToPosition(RED_DEPOT_MARKER [X], RED_DEPOT_MARKER [Y], .5);
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
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
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(RED_DEPOT_CENTER[X], RED_DEPOT_CENTER[Y], .5);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-700);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    gridNavigation.driveToPosition(.75, 1.5, .5);
                    gridNavigation.driveToPosition(.5, 2.5, .5);
                    gridNavigation.driveToPosition(RED_DEPOT_MARKER [X], RED_DEPOT_MARKER [Y], .5);
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
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