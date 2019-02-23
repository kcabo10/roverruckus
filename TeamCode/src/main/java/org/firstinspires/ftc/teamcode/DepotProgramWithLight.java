package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our autonomous program for the depot side on both sides of the field. This program runs
 * with the phone light for Tensor Flow. Use this program if the lighting is really dim or light
 * at all. This programs lands, hits off the gold mineral, deposits the team marker, and parks in the
 * other alligances crater.
 */
@Autonomous(name = "Depot Program With Light", group = "Beep")
public class DepotProgramWithLight extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    //Calling our hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calling the Library Gyro program to use the gyro turn function
    LibraryGyro gyroTurn = new LibraryGyro();
    // Calling the Library Gyro Drive program to use the gyro drive function
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    // Calling the Library Grid Nav Library to use the grid navigation functions
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    // Calling the Library Tensor Flow No Light to use the Tensor Flow function with
    // initializing the light
    LibraryTensorFlowObjectDetectionWithLight tensorFlow =
            new LibraryTensorFlowObjectDetectionWithLight(robot, telemetry);
    // Declaring gold position value to read what position Tensor Flow sees the gold mineral in
    String goldPosition = "";

    /**
     * @throws InterruptedException <p>
     *                              The main body of our code which contains the set of commands carried out in our crater side autonomous program.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        //initializing the hardware map
        robot.init(hardwareMap);
        //initializing the grid Nav function
        gridNavigation.init(robot, gyroTurn, telemetry);
        //initializing the gyro turn function
        gyroTurn.init(robot, telemetry);
        //initializing the gyro drive function
        gyroDrive.init(robot, telemetry, robot.rightBack);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        // setting initial latch power to 0
        robot.latch.setPower(0);

        //wait for start
        waitForStart();

        // landing our robot
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(-20680);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

        // Start up Tensor Flow to read mineral position while landing
        getMineralPosition();

        // stop running lift once it gets to target position
        robot.lift.setPower(0);
        runtime.reset();
        // run servo to open latch
        robot.latch.setPower(-1);

        //wait for 1.15 seconds
        while (runtime.seconds() < 1.15) {

        }
        // set servo power to 0
        robot.latch.setPower(0);
        robot.lift.setPower(0);
        runtime.reset();

        // took the hypotenuse line to the center of the robot is 20.625 inches
        // Set initial Grid Nav position
        gridNavigation.setGridPosition(.6076, .6076, 45);

        int X = 0;
        int Y = 1;

        // Left mineral pos
        double[] RED_DEPOT_LEFT = {1.2, 2.4};
        // right mineral pos
        double[] RED_DEPOT_RIGHT = {1.2916, .9114};
        // center mineral pos
        double[] RED_DEPOT_CENTER = {1.1, 1.1};

        // right and center marker pos
        double[] RED_DEPOT_MARKER = {1.6, 2.6};
        // left mineral marker pos
        double[] LEFT_DEPOT_MARKER = {1.6, 2.4};

        // Parking pos for all mineral positions
        double[] RED_DEPOT_PARKING = {-.8, 2.65};

        // This is a switch block that plays the program in relation to the mineral position that
        // Tensor Flow reads
        switch (goldPosition) {

            // If Tensor Flow reads the left mineral position then it plays this case
            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);
                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(.8, .8, .5);
                    lowerLift();
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);
                    gridNavigation.driveToPosition(LEFT_DEPOT_MARKER[X], LEFT_DEPOT_MARKER[Y], .5);
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
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
//                    gridNavigation.driveToPosition(RED_DEPOT_ARM_DROP[X], RED_DEPOT_ARM_DROP[Y], .5);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }

                break;

            // If Tensor Flow reads the right mineral position then it plays this case
            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);
                if (goldPosition == "RIGHT") {
                    // drive away from lander
                    gridNavigation.driveToPosition(.8, .8, .5);
                    // start lower lift
                    lowerLift();
                    // drop arm
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(750);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    // drive to right mineral pos
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
                    // lift arm
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-825);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    // set power to .111 to keep arm up
                    robot.arm.setPower(.111);
                    // drive around minerals to get to depot
                    gridNavigation.driveToPosition(.5, 1.5, .5);
                    gridNavigation.driveToPosition(.5, 2.5, .5);
                    // drive towards depot to deposit marker
                    gridNavigation.driveToPosition(RED_DEPOT_MARKER[X], RED_DEPOT_MARKER[Y], .5);
                    // bring arm down
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(450);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    // hold arm position
                    robot.arm.setPower(.111);
                    // run intake to deposit marker
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    // bring arm up
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-450);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    // drive to crater to park
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    // bring arm down to park
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

            // If Tensor Flow reads the center mineral position then it plays this case
            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                printTelemetry(60);
                if (goldPosition == "CENTER") {
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_CENTER[Y]);
                    // bring arm down
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(750);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    // drive to center mineral position
                    gridNavigation.driveToPosition(RED_DEPOT_CENTER[X], RED_DEPOT_CENTER[Y], .5);
                    // start lowering the lift
                    lowerLift();
                    // bring arm up
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    // drive around right mineral to get to depot
                    gridNavigation.driveToPosition(.65, 1.5, .5);
                    gridNavigation.driveToPosition(.2, 2.5, .5);
                    // drive to depot to deposit marker
                    gridNavigation.driveToPosition(RED_DEPOT_MARKER[X], RED_DEPOT_MARKER[Y], .5);
                    // bring arm down
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(450);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    // hold arm position
                    robot.arm.setPower(.111);
                    // run intake to deposit intake
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    // bring arm up
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-450);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                    // drive to crater to park
                    gridNavigation.driveToPosition(RED_DEPOT_PARKING[X], RED_DEPOT_PARKING[Y], .7);
                    // lower arm to park in crater
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(800);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(.111);
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(70);
                }

                telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);

                telemetry.update();
                break;
            // should never get to this case but in case it can't find the mineral position
            // it goes to this default case
            default:
                telemetry.addData("Telemetry", "Didn't see gold pos");
                telemetry.update();
                break;
        }

        // Once it goes through the case block it does the following
        telemetry.addData("Parked Ready to pull out arm", "");
        telemetry.update();

        // We stop using Grid Nav at this point and drive backwards to pull out our arm
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

        // Wait until wheels encoders have gone to -537
        while (robot.rightFront.isBusy()) {

        }
        // Shut off motors
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);

        // Lift up arm
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(-400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);
        while (robot.arm.isBusy()) {
        }
        robot.arm.setPower(.111);

        // driving forward
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

        // Bringing the arm down
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);
        while (robot.arm.isBusy()) {
        }
        robot.arm.setPower(.111);

        // Start intake to suck in minerals
        robot.intake.setPower(1);
        sleep(1000);
        robot.intake.setPower(0);

        // Continuous while block
        while (true) {

            //start intake
            robot.intake.setPower(1);

            // driving backwards
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

            // driving forward
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

            // Stopping intake
            robot.intake.setPower(0);
        }
    }

    /**
     * @param codePos This is the value we use in telemetry to see where in the code we are
     */
    private void printTelemetry(int codePos) {
        telemetry.addData("Gold Pos", goldPosition);
        telemetry.addData("Code Position", codePos);
        telemetry.update();
    }

    /**
     * Lowers lift while reading the sampling
     */
    private void lowerLift() {
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(14500);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

    }

    /**
     * Calls Tensor Flow in order to read gold mineral position
     */
    public void getMineralPosition() {
        int debounceCount = 0;
        long startTime = 0;
        String previousPosition;
        goldPosition = tensorFlow.findMineral();

        // Switch block that indicated which mineral position it reads
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

            // If it reads unknown than it goes to this default case
            default:
                telemetry.addData("Telemetry", "Unknown Position");
                telemetry.update();
                // sets mineral pos to center as default
                goldPosition = "CENTER";
                break;
        }

        telemetry.update();
    }
}