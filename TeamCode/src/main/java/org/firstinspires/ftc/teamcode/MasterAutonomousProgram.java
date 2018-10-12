package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name="Master Autonomous Program", group="Beep")
public class MasterAutonomousProgram extends LinearOpMode {

    HardwareBeep robot   = new HardwareBeep();
    LibraryGyro gyro = new LibraryGyro();
    LibraryVuMarkIdentification vuMark = null;

    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.3;


    public String foundTargetName = "";

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        sleep(2000);
        robot.init(hardwareMap);
        gyro.init(robot, telemetry);

        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        vuMark = new LibraryVuMarkIdentification(hardwareMap, telemetry);

        // wait for start button.

        waitForStart();


        foundTargetName = vuMark.getTarget();
        telemetry.addData("Found Target is ", foundTargetName);
        telemetry.update();


        switch(foundTargetName){
            case "Blue-Rover":
            case "Red-Footprint":
                telemetry.addData("Telemetry","Crater Program");
                //Land robot
                //Set motors to zero
                gyro.turnGyro(90);
                break;
            case "Front-Craters":
            case "Back-Space":
                telemetry.addData("Telemetry","Depot Program");
                //Land robot
                //Set motors to zero
                gyro.turnGyro(-90);
                break;
            default:
                telemetry.addData("Telemetry","No vuMark found");
                //throw new IllegalArgumentException("Unknown vuMark: " + foundTargetName);
        }
        telemetry.update();
        sleep(10000);
    }
}