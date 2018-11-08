package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by kyliestruth 10/5/17.
 */

@TeleOp(name= "TeleOp Program", group= "TankDrive")
public class TeleOpProgram extends OpMode
{
    private HardwareBeep robot = new HardwareBeep();

    private int buttonYPressed;
    private int buttonAPressed;
    private int direction = 1;
    private double scaleFactor = 0.5;

    public void reverseDirection() {
        if (direction == 1) {
            direction = -1;
        } else if (direction == -1) {
            direction = 1;
        }
    }

    public void scaleFactor(){
        if (scaleFactor == 1) {
            scaleFactor = 0.5;
        } else if (scaleFactor == 0.5) {
            scaleFactor = 1;
        }
    }
     @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");

    }

    public void init_loop() {

        buttonYPressed = 0;
        robot.lift.setPower(0);
        robot.arm.setPower(0);
        robot.armExtrusion.setPower(0);
        robot.intake.setPower(0);
//        robot.basket.setPosition(.45);
        robot.latch.setPower(-.1);
    }

    public void loop() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        /**
         *POV Mecanum Wheel Control
         */
        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;

        left  = drive + turn;
        right = drive - turn;

        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        left = left * scaleFactor * direction;

        right = right * scaleFactor * direction;

        robot.leftFront.setPower(left);
        robot.leftBack.setPower(left);
        robot.rightFront.setPower(right);
        robot.rightBack.setPower(right);

        /**
         *Invert Direction On Y Button
         */

        switch (buttonYPressed){
            case(0):
                if (gamepad1.y) {
                    buttonYPressed = 1;
                }
                break;
            case(1):
                if (!gamepad1.y) {
                    buttonYPressed = 0;
                    reverseDirection();
                }
                break;
        }

        /**
         *ScaleFactor on A Button
         */


        switch (buttonAPressed) {
            case(0):
             if (gamepad1.a)  {
                 buttonAPressed = 1;
             }
            break;
            case(1):
                if (!gamepad1.a) {
                    buttonAPressed = 0;
                    scaleFactor();
                }
            break;
        }

        /**
         *Lift Control
         */
        if (gamepad2.dpad_down && !gamepad2.dpad_up)        {

            robot.lift.setPower(1);
            telemetry.addData("Encoder Ticks", robot.lift.getCurrentPosition());
            telemetry.update();
        } else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.lift.setPower(-1);
            telemetry.addData("Encoder Ticks", robot.lift.getCurrentPosition());
            telemetry.update();
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.lift.setPower(0);
        }

        /**
         Latch Release Control
          */

        if (gamepad2.dpad_right && !gamepad2.dpad_left)        {

            robot.latch.setPower(1);
        } else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.latch.setPower(-1);
        } else if (!gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.latch.setPower(-.1);
        }


        /**
         *Intake Control
         */
        if (gamepad2.right_bumper) {
            robot.intake.setPower(0.3);
        } else if (gamepad2.right_trigger > 0) {
            robot.intake.setPower(-0.3);
        } else
            robot.intake.setPower(0);

        /**
         *Basket Control
         */
//        if (gamepad2.left_bumper) {
//            robot.basket.setPosition(1);
//        } else if (gamepad2.left_trigger > 0.45) {
//            robot.basket.setPower(-1);
//        } else
//            robot.basket.setPower(0.45);

        /**
         *Arm Control
         */

        if (gamepad2.left_stick_y > 0) {
            robot.arm.setPower(0.3);
        } else if (gamepad2.left_stick_y < 0)
            robot.arm.setPower(-0.3);
        else if (gamepad2.left_stick_y == 0)
            robot.arm.setPower(0);

        /**
        Arm Extrusions
         */

        if (gamepad2.right_stick_y > 0) {
            robot.armExtrusion.setPower(0.3);
        } else if (gamepad2.right_stick_y < 0)
            robot.armExtrusion.setPower(-0.3);
        else if (gamepad2.right_stick_y == 0)
            robot.armExtrusion.setPower(0);


        telemetry.addData("Commanded left motor power", left);
        telemetry.addData("Commanded right motor power", right);
        telemetry.addData("Scale Factor", scaleFactor);
        telemetry.addData("Direction", direction);
        telemetry.addData("left front power", robot.leftFront.getPower());
        telemetry.addData("left back power", robot.leftBack.getPower());
        telemetry.addData("right front power", robot.rightFront.getPower());
        telemetry.addData("right back power", robot.rightBack.getPower());
        telemetry.update();
    }
    
    public void stop() {

        robot.lift.setPower(0);
        robot.arm.setPower(0);
        robot.armExtrusion.setPower(0);
        robot.intake.setPower(0);
//        robot.basket.setPosition(.45);
        robot.latch.setPower(-.1);
    }
}