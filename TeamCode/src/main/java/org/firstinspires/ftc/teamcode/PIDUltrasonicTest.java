package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.sensors.SensorMB1242;

@TeleOp(name="PID Ultrasonic Test", group="Test")
//@Disabled
public class PIDUltrasonicTest extends LinearOpMode {
    HardwareBeep robot = new HardwareBeep();
    LibraryUltrasonicDrive ultrasonicDrive = new LibraryUltrasonicDrive();
    boolean last = false;
    public ElapsedTime runtime = new ElapsedTime();
    int i = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("waitForStart()", i++);
        telemetry.update();
        ultrasonicDrive.turnUltrasonic(20000);

        telemetry.addData("Should have ran turnUltrasonic", "");
        telemetry.update();

        }
}