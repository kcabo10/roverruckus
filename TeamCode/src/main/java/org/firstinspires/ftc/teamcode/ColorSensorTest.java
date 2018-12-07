package org.firstinspires.ftc.teamcode;

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

        waitForStart();

        robot.color_Sensor.red();
        robot.color_Sensor.blue();


        robot.color_Sensor.enableLed(true);

        if (robot.color_Sensor.blue() < 20) {
            robot.latch.setPower(1);
        }

    }
}
