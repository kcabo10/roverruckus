package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Depot Program", group="Beep")
public class DepotProgram extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    LibraryDogeforia dogeforia = new LibraryDogeforia(robot, telemetry);
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryTensorFlowObjectDetection tensorFlow = new LibraryTensorFlowObjectDetection(robot, telemetry);

    String goldPosition = "";



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


        /**
        Wait for start button.
         */

        waitForStart();

        // landing our robot

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

        gridNavigation.setGridPosition(.5, -.5, -45);

        int codePos = 0;

        getMineralPosition();

        //goldPosition = "RIGHT";
        printTelemetry(10);
        telemetry.update();

        //telemetry.update();

        sleep(3000);

        gridNavigation.driveToPosition(.7, 0.7, .2);

        int X = 0;
        int Y = 1;

       /**
        Change values to grab mineral
        */


        double[] BLUE_DEPOT_LEFT = {-2, -1};
        double[] BLUE_DEPOT_RIGHT = {-1, -2};
        double[] BLUE_DEPOT_CENTER = {-1.5, -1.5};

        double[] RED_DEPOT_LEFT = {2, -1};
        double[] RED_DEPOT_RIGHT = {1, -2};
        double[] RED_DEPOT_CENTER = {1.5, -1.5};

//        Change values to park

        double[] BLUE_DEPOT_LEFT_PARKING = {0, 0};
        double[] BLUE_DEPOT_RIGHT_PARKING = {0, 0};
        double[] BLUE_DEPOT_CENTER_PARKING = {0, 0};

        double[] RED_DEPOT_LEFT_PARKING = {0, 0};
        double[] RED_DEPOT_RIGHT_PARKING = {0, 0};
        double[] RED_DEPOT_CENTER_PARKING = {0, 0};

        switch (goldPosition) {

            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);

                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);
//                    gridNavigation.driveToPosition(RED_DEPOT_LEFT_PARKING[X], RED_DEPOT_LEFT_PARKING[Y], .2);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }



            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);

                if (goldPosition == "RIGHT"){
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
//                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT_PARKING[X], RED_DEPOT_RIGHT_PARKING[Y], .2);
                }
                else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }



            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                printTelemetry(60);

                if (goldPosition == "CENTER"){
                    gridNavigation.driveToPosition(RED_DEPOT_CENTER[X], RED_DEPOT_CENTER[Y], .2);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_CENTER[Y]);
//                    gridNavigation.driveToPosition(RED_DEPOT_CENTER_PARKING[X], RED_DEPOT_CENTER_PARKING[Y], .2);
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