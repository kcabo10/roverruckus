package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;


@Autonomous(name="Master Autonomous Program", group="Beep")
public class MasterAutonomousProgram extends LinearOpMode {

    HardwareBeep robot   = new HardwareBeep();
    LibraryGyro gyro = new LibraryGyro();
    LibraryDogeforia dogeforia = new LibraryDogeforia(robot.hwMap, telemetry);
    //Dogeforia vuforia;


    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;


    public String foundTargetName = "";


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        robot.init(hardwareMap);
        gyro.init(robot, telemetry);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();
        dogeforia.init();



        // wait for start button.

        waitForStart();

        gyro.turnGyro(45);

        foundTargetName = dogeforia.loop();
        telemetry.addData("Found Target is ", foundTargetName);
        telemetry.update();
        sleep(2000);


        switch(foundTargetName){
            case "Blue-Rover":
            case "Red-Footprint":
                telemetry.addData("Telemetry","Crater Program");
                //Land robot
                //Set motors to zero
                gyro.turnGyro(-45);
                getMineralPosition();
                break;
            case "Front-Craters":
            case "Back-Space":
                telemetry.addData("Telemetry","Depot Program");
                //Land robot
                //Set motors to zero
                gyro.turnGyro(-45);
                getMineralPosition();
                break;
            default:
                telemetry.addData("Telemetry","No vuMark found");

        }
        telemetry.update();
        sleep(2000);

    }
    public void getMineralPosition(){
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

        String myPosition = null;
        myPosition = detector.getLastOrder().toString();
        telemetry.addData("MAP", "Running Sampling Order loop")
        while (myPosition == null) {

            telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
            telemetry.addData("Last Order" , detector.getLastOrder().toString()); // The last known result

            myPosition = detector.getLastOrder().toString();

            switch(myPosition){
                case("LEFT"):
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
                case("UNKNOWN"):
                    telemetry.addData("Telemetry", "Unknown Position");
                    telemetry.update();
                    break;
            }
            telemetry.update();
        }

    }
}