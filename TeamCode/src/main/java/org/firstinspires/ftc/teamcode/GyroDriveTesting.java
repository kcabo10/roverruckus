package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="GyroDriveTesting", group="Exercises")
public class GyroDriveTesting extends LinearOpMode {

    public HardwareBeep robot   = new HardwareBeep();
    public LibraryGyro gyro     = new LibraryGyro();
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyro.init(robot, telemetry);
        gyroDrive.init(robot, telemetry, robot.leftFront);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyroTurn is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        gyroDrive.driveGyro(.3, 10000);
        sleep(3000);

    }
}