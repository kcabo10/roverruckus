package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by kyliestruth 10/27/18.
 */

@Autonomous(name= "Color Sensor Test", group= "TankDrive")
public class ColorSensorTest extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};

        waitForStart();

        Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);

        robot.colorSensor.enableLed(true);

        telemetry.addData("Phase", "Hasn't entered do statement");
        telemetry.update();

        do {
            telemetry.addData("Phase", "Do Statement");
            telemetry.update();
            robot.latch.setPower(1);
            } while (robot.colorSensor.blue() < robot.colorSensor.red());

        telemetry.addData("Phase", "While Loop complete");
        telemetry.addData("found blue Color", robot.colorSensor.blue());
        telemetry.update();
        sleep(5000);
    }
}
