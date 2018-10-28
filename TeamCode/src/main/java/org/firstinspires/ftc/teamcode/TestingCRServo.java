package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by kyliestruth 10/27/18.
 */

@TeleOp(name= "TestingCRServo", group= "TankDrive")
public class TestingCRServo extends OpMode
{
    private HardwareMapTestingCRServo robot = new HardwareMapTestingCRServo();
     @Override
    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    public void init_loop() {

         robot.basket.setPosition(0);
    }

    public void loop() {

         robot.basket.setPosition(.50);
    }

    public void stop() {

        robot.basket.setPosition(0);
    }
}
