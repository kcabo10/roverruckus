package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Master Autonomous Program", group="Beep")
public class MasterAutonomousProgram extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryDogeforia dogeforia = new LibraryDogeforia(robot, telemetry);
    LibraryVuMarkIdentification vuforia;
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    String goldPosition = "";


    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;


    public String foundTargetName = "";

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
        vuforia = new LibraryVuMarkIdentification(robot.hwMap, telemetry);


        /**
        Wait for start button.
         */

        waitForStart();

        /**
         landing our robot
        */

//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lift.setTargetPosition(-12500);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lift.setPower(1);
//
//        while (opModeIsActive() &&
//                robot.lift.isBusy()){
//            telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
//            telemetry.update();
//
//        }
//
//        robot.lift.setPower(0);

        /**
        Setting code position
         */

        int codePos = 0;

        /**
         Getting the mineral position
         */

        getMineralPosition();

        /**
         Printing telemetry
         */

        printTelemetry(10);

        sleep(2000);

        telemetry.addData("Turning", telemetry);
        telemetry.update();

        /**
         Turning robot 45 degrees and driving forward 800 encoder ticks
         */

        gyroTurn.turnGyro(45);
        gyroDrive.driveGyro(.3, -800);//1611);

        /**
         Printing telemetry
         */

        printTelemetry(20);

        /**
         Reading vuMark
         */

        foundTargetName = vuforia.getTarget();

        /**
         Getting grid position from vuMark
         */
        gridNavigation.setGridPosition(vuforia.gridPos[0], vuforia.gridPos[1], vuforia.rotation.thirdAngle);

        /**
         Printing telemetry
         */

        telemetry.addData("Grid Nav Set Pos X", vuforia.gridPos[0]);
        telemetry.addData("Grid Nav Set Pos Y", vuforia.gridPos[1]);

        printTelemetry(30);


        //telemetry.update();

        sleep(3000);

        /**
         Setting x and y
          */

        int X = 0;
        int Y = 1;

        /**
         Change values to grab mineral
         */

        double[] BLUE_DEPOT_LEFT = {-2, 1};
        double[] BLUE_DEPOT_RIGHT = {-1, 2};
        double[] BLUE_DEPOT_CENTER = {-1.5, 1.5};

        double[] BLUE_CRATER_LEFT = {1, 2};
        double[] BLUE_CRATER_RIGHT = {2, 1};
        double[] BLUE_CRATER_CENTER = {1.5, 1.5};

        double[] RED_DEPOT_LEFT = {2, -1};
        double[] RED_DEPOT_RIGHT = {1, -2};
        double[] RED_DEPOT_CENTER = {1.5, -1.5};

        double[] RED_CRATER_LEFT = {-1, -2};
        double[] RED_CRATER_RIGHT = {-2, -1};
        double[] RED_CRATER_CENTER = {-1.5, -1.5};

        /**
        Change values to park
         */

        double[] BLUE_DEPOT_LEFT_PARKING = {0, 0};
        double[] BLUE_DEPOT_RIGHT_PARKING = {0, 0};
        double[] BLUE_DEPOT_CENTER_PARKING = {0, 0};

        double[] BLUE_CRATER_LEFT_PARKING = {0, 0};
        double[] BLUE_CRATER_RIGHT_PARKING = {0, 0};
        double[] BLUE_CRATER_CENTER_PARKING = {0, 0};

        double[] RED_DEPOT_LEFT_PARKING = {0, 0};
        double[] RED_DEPOT_RIGHT_PARKING = {0, 0};
        double[] RED_DEPOT_CENTER_PARKING = {0, 0};

        double[] RED_CRATER_LEFT_PARKING = {0, 0};
        double[] RED_CRATER_RIGHT_PARKING = {0, 0};
        double[] RED_CRATER_CENTER_PARKING = {0, 0};

        /**
         Determining target position and following set commands
         */

        switch (foundTargetName) {
            case "Red-Footprint":
                telemetry.addData("Telemetry", "Red Crater Program");
                printTelemetry(40);

                if (goldPosition == "LEFT") {
                    /**
                     Drives to the red crater left position using preset grid positions
                     */
                    gridNavigation.driveToPosition(RED_CRATER_LEFT[X], RED_CRATER_LEFT[Y], .2);
//                    gridNavigation.driveToPosition(RED_CRATER_LEFT_PARKING[X], RED_CRATER_LEFT_PARKING[Y], .2);
                }
                if (goldPosition == "RIGHT"){
                    /**
                     Drives to the red crater right position using preset grid positions
                     */
                    gridNavigation.driveToPosition(RED_CRATER_RIGHT[X], RED_CRATER_RIGHT[Y], .2);
//                    gridNavigation.driveToPosition(RED_CRATER_RIGHT_PARKING[X], RED_CRATER_RIGHT_PARKING[Y], .2);
                }
                if (goldPosition == "CENTER"){
                    /**
                     Drives to the red crater center position using preset grid positions
                     */
                    gridNavigation.driveToPosition(RED_CRATER_CENTER[X], RED_CRATER_CENTER[Y], .2);
//                    gridNavigation.driveToPosition(RED_CRATER_CENTER_PARKING[X], RED_CRATER_CENTER_PARKING[Y], .2);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                }


            case "Blue-Rover":
                telemetry.addData("Telemetry", "Blue Crater Program");
                printTelemetry(50);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(BLUE_CRATER_LEFT[X], BLUE_CRATER_LEFT[Y], .2);
//                    gridNavigation.driveToPosition(BLUE_CRATER_LEFT_PARKING[X], BLUE_CRATER_LEFT_PARKING[Y],.2);
                }
                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(BLUE_CRATER_RIGHT[X], BLUE_CRATER_RIGHT[Y], .2);
//                    gridNavigation.driveToPosition(BLUE_CRATER_RIGHT_PARKING[X], BLUE_CRATER_RIGHT_PARKING[Y], .2);
                }
                if (goldPosition == "CENTER"){
                        gridNavigation.driveToPosition(BLUE_CRATER_CENTER[X], BLUE_CRATER_CENTER[Y], .2);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                }
                    break;


            case "Front-Craters":
                telemetry.addData("Telemetry", "Blue Depot Program");
                printTelemetry(60);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(BLUE_DEPOT_LEFT[X], BLUE_DEPOT_LEFT[Y], .2);
//                    gridNavigation.driveToPosition(BLUE_DEPOT_LEFT_PARKING[X], BLUE_DEPOT_LEFT_PARKING[Y], .2);
                }
                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(BLUE_DEPOT_RIGHT[X], BLUE_DEPOT_RIGHT[Y], .2);
//                    gridNavigation.driveToPosition(BLUE_DEPOT_RIGHT_PARKING[X], BLUE_DEPOT_RIGHT_PARKING[Y], .2);
                }
                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(BLUE_DEPOT_CENTER[X], BLUE_DEPOT_CENTER[Y], .2);
//                    gridNavigation.driveToPosition(BLUE_DEPOT_CENTER_PARKING[X], BLUE_DEPOT_CENTER_PARKING[Y], .2);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                }


            case "Back-Space":
                telemetry.addData("Telemetry", "Red Depot Program");
                telemetry.addData("Grid Nav Set Pos X", vuforia.gridPos[0]);
                telemetry.addData("Grid Nav Set Pos Y", vuforia.gridPos[1]);
                printTelemetry(70);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);
//                    gridNavigation.driveToPosition(RED_DEPOT_LEFT_PARKING[X], RED_DEPOT_LEFT_PARKING[Y], .2);
                }
                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
//                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT_PARKING[X], RED_DEPOT_RIGHT_PARKING[Y], .2);
                }
                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(RED_DEPOT_CENTER[X], RED_DEPOT_CENTER[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_CENTER[Y]);
//                    gridNavigation.driveToPosition(RED_DEPOT_CENTER_PARKING[X], RED_DEPOT_CENTER_PARKING[Y], .2);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(80);
                }

                telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);

                telemetry.update();
                sleep (2000);
                break;


            default:
                telemetry.addData("Telemetry", "No vuMark found");

        }
        //foundTargetName, mineral postion,
        //code pos

        //telemetry.update();


    }

    /**
     Print telemetry function
     */

    private void printTelemetry(int codePos) {
        telemetry.addData("Target Found", foundTargetName);
        telemetry.addData("Gold Pos", goldPosition);
        telemetry.addData("Code Position", codePos);
        telemetry.update();
    }

    /**
     Get mineral position function
     */

    public void getMineralPosition() {

        /**
         Debounce algoritm to detect mineral accurately
         */

        int debounceCount = 0;
        long startTime = 0;
        startTime = System.currentTimeMillis();

        /**
         Initializing dogeCV for getting mineral position
         */

        String previousPosition;
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        LibrarySamplingOrderDetector detector = new LibrarySamplingOrderDetector();
        detector.init(robot.hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        goldPosition = "UNKNOWN";
        previousPosition = "UNKNOWN";
        telemetry.addData("MAP", "Running Sampling Order loop");
        goldPosition = detector.getCurrentOrder().toString();

        /**
         This while function is waiting until it has seen the correct mineral position by finding the value of one of the mineral position then debouncing the mineral position until it settles at the correct mineral position.
         */
        while (goldPosition == "UNKNOWN") {

            gyroTurn.turnGyro(-5);

            sleep(5000);

            startTime = System.currentTimeMillis();


            while (System.currentTimeMillis() < (startTime + 10000)) {

                goldPosition = detector.getCurrentOrder().toString();

                if (goldPosition == previousPosition) {
                }
                else {
                    previousPosition = goldPosition;
                    startTime = System.currentTimeMillis();

                }
                telemetry.addData("StartTime: ", startTime);
                telemetry.addData("CurrentTime: ", System.currentTimeMillis());
                telemetry.addData("Prev Position:  ", previousPosition);
                telemetry.addData("Gold Position:  ", goldPosition);
                telemetry.update();
            }


            telemetry.addData("Current Order", detector.getCurrentOrder().toString()); // The current result for the frame
            telemetry.addData("Last Order", detector.getLastOrder().toString()); // The last known result

            /**
             Telemetry for gold position
             */

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
        detector.disable();

    }
}