package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Crater Program No Light", group="Beep")
public class CraterProgramJustParkingNoLight extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryTensorFlowObjectDetectionNoLight tensorFlow = new LibraryTensorFlowObjectDetectionNoLight(robot, telemetry);

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

        robot.lift.setPower(0);
        runtime.reset();
        robot.latch.setPower(-1);

        while (runtime.seconds() <1.15){

        }
        robot.latch.setPower(0);
        robot.lift.setPower(0);
        runtime.reset();

        gridNavigation.setGridPosition(.8281, .8281, 45);
        printTelemetry(10);
        telemetry.update();
        gridNavigation.driveToPosition(1, 1, .7);

        int X = 0;
        int Y = 1;

       /**
        Change values to grab mineral
        */

        double[] RED_CRATER_LEFT = {1.2, 2.2};
        double[] RED_CRATER_RIGHT = {2.2, 1.2};
        double[] RED_CRATER_CENTER = {1.5, 1.5};

        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_CRATER_LEFT[X], RED_CRATER_LEFT[Y], .7);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_LEFT[Y]);
                    printTelemetry(20);
                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lift.setTargetPosition(16000);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lift.setPower(1);

                    runtime.reset();
                    robot.latch.setPower(1);
                    while (runtime.seconds() < 1.15) {
                    }
                    robot.latch.setPower(0);

                    gyroTurn.turnGyro(-40);

                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }
            break;

            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);

                if (goldPosition == "RIGHT") {
                    gridNavigation.driveToPosition(RED_CRATER_RIGHT[X], RED_CRATER_RIGHT[Y], .7);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_RIGHT[Y]);
                    printTelemetry(40);
                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lift.setTargetPosition(16000);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lift.setPower(1);

                    runtime.reset();
                    robot.latch.setPower(1);
                    while (runtime.seconds() < 1.15) {
                    }
                    robot.latch.setPower(0);

                    gyroTurn.turnGyro(40);

                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }
            break;

            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                printTelemetry(60);

                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(RED_CRATER_CENTER[X], RED_CRATER_CENTER[Y], .7);
                    telemetry.addData("Grid Nav Goto Pos X", RED_CRATER_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_CRATER_CENTER[Y]);
                    printTelemetry(60);
                    robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.lift.setTargetPosition(16000);
                    robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.lift.setPower(1);

                    runtime.reset();
                    robot.latch.setPower(1);
                    while (runtime.seconds() < 1.15) {
                    }
                    robot.latch.setPower(0);

                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-720);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
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