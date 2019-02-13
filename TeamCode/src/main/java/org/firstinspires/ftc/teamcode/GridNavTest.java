package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Grid Nav Test", group="Test")
//@Disabled
public class GridNavTest extends LinearOpMode {
    HardwareBeep robot = new HardwareBeep();
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    LibraryGyro gyroTurn = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    boolean last = false;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        gridNavigation.init(robot, gyroTurn, telemetry);
        gyroTurn.init(robot, telemetry);
        gyroDrive.init(robot, telemetry, robot.rightBack);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("waitForStart()", "");
        telemetry.update();

        gridNavigation.setGridPosition(.8281, .8281, 45);
        gridNavigation.driveToPosition(1,1, .7);
        telemetry.addData("Should have ran to grid pos", "");
        telemetry.update();

        }
}