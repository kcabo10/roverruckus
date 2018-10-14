package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name="Master Autonomous Program", group="Beep")
public class MasterAutonomousProgram extends LinearOpMode {

    HardwareBeep robot   = new HardwareBeep();
    LibraryGyro gyro = new LibraryGyro();
    LibraryDogeforia dogeforia = null;
    //Dogeforia vuforia;


    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;


    public String foundTargetName = "";

    private LibrarySamplingOrderDetector detector = new LibrarySamplingOrderDetector();


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        robot.init(hardwareMap);
        gyro.init(robot, telemetry);
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        dogeforia = new LibraryDogeforia(hardwareMap, telemetry);

        // wait for start button.

        waitForStart();

        gyro.turnGyro(45);

        foundTargetName = dogeforia.loop();
        telemetry.addData("Found Target is ", foundTargetName);
        telemetry.update();


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

        detector = new LibrarySamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
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