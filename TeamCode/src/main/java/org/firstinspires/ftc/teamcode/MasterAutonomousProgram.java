package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name="Master Autonomous Program", group="Beep")
public class MasterAutonomousProgram extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    DriveAvoidIMU gyroDrive = new DriveAvoidIMU();
    LibraryDogeforia dogeforia = new LibraryDogeforia(robot, telemetry);
    LibraryVuMarkIdentification vuforia;
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();

    String goldPosition = "";


    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;


    public String foundTargetName = "";

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        robot.init(hardwareMap);
        gridNavigation.init(robot, gyroTurn, telemetry);
        gyroTurn.init(robot, telemetry);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();
        //dogeforia.init();
        vuforia = new LibraryVuMarkIdentification(robot.hwMap, telemetry);


        // wait for start button.

        waitForStart();

        getMineralPosition();
        sleep(500);

        gyroTurn.turnGyro(45);

        foundTargetName = vuforia.getTarget();
        gridNavigation.setGridPosition(vuforia.gridPos[0], vuforia.gridPos[1], vuforia.rotation.thirdAngle);

        telemetry.addData("MAP", "Entering Vuf Switch");

        telemetry.addData("Grid Nav Set Pos X", vuforia.gridPos[0]);
        telemetry.addData("Grid Nav Set Pos Y", vuforia.gridPos[1]);


        telemetry.update();

        sleep(3000);

        int X = 0;
        int Y = 1;

//        Change values to grab mineral


        double[] BLUE_DEPOT_LEFT = {-2, 1};
        double[] BLUE_DEPOT_RIGHT = {-1, 2};
        double[] BLUE_DEPOT_CENTER = {-1.5, 1.5};

        double[] BLUE_CRATER_LEFT = {1, 2};
        double[] BLUE_CRATER_RIGHT = {2, 1};
        double[] BLUE_CRATER_CENTER = {1.5, 1.5};

        double[] RED_DEPOT_LEFT = {4, -2};
        double[] RED_DEPOT_RIGHT = {2, -4};
        double[] RED_DEPOT_CENTER = {3, -3};

        double[] RED_CRATER_LEFT = {-1, -2};
        double[] RED_CRATER_RIGHT = {-2, -1};
        double[] RED_CRATER_CENTER = {-1.5, -1.5};

        switch (foundTargetName) {
            case "Red-Footprint":
                telemetry.addData("Telemetry", "Red Crater Program");

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_CRATER_LEFT[X], RED_CRATER_LEFT[Y], .1);
                }
                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(RED_CRATER_RIGHT[X], RED_CRATER_RIGHT[Y], .1);
                }
                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(RED_CRATER_CENTER[X], RED_CRATER_CENTER[Y], .1);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                }


            case "Blue-Rover":
                telemetry.addData("Telemetry", "Blue Crater Program");

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(BLUE_CRATER_LEFT[X], BLUE_CRATER_LEFT[Y], .2);
                }
                if (goldPosition == "RIGHT"){
                        gridNavigation.driveToPosition(BLUE_CRATER_RIGHT[X], BLUE_CRATER_RIGHT[Y], .2);
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

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(BLUE_DEPOT_LEFT[X], BLUE_DEPOT_LEFT[Y], .2);
                }
                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(BLUE_DEPOT_RIGHT[X], BLUE_DEPOT_RIGHT[Y], .2);
                }
                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(BLUE_DEPOT_CENTER[X], BLUE_DEPOT_CENTER[Y], .2);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                }


            case "Back-Space":
                telemetry.addData("Telemetry", "Red Depot Program");
                telemetry.addData("Grid Nav Set Pos X", vuforia.gridPos[0]);
                telemetry.addData("Grid Nav Set Pos Y", vuforia.gridPos[1]);


                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);
                }
                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
                }
                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(RED_DEPOT_CENTER[X], RED_DEPOT_CENTER[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_CENTER[Y]);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                }

                telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);

                telemetry.update();
                sleep (2000);
                break;


            default:
                telemetry.addData("Telemetry", "No vuMark found");

        }
        telemetry.update();


    }

    public void getMineralPosition() {
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



        goldPosition = "RIGHT";
        telemetry.addData("MAP", "Running Sampling Order loop");
        while (goldPosition == "UNKNOWN") {


            goldPosition = detector.getCurrentOrder().toString();

            telemetry.addData("Current Order", detector.getCurrentOrder().toString()); // The current result for the frame
            telemetry.addData("Last Order", detector.getLastOrder().toString()); // The last known result


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