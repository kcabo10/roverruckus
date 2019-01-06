package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by kyliestruth 10/5/17.
 */

@TeleOp(name= "TeleOp Program", group= "TankDrive" )
public class TeleOpProgram extends OpMode
{
    private HardwareBeep robot = new HardwareBeep();

    private int buttonYPressed;
    private int buttonAPressed;
    private int buttonAPressedG2;
    private int direction = -1;
    private double scaleFactor = 1;

    public void reverseDirection() {
        if (direction == 1) {
            direction = -1;
        } else if (direction == -1) {
            direction = 1;
        }
    }

    public void scaleFactor(){
        if (scaleFactor == 0.5) {
            scaleFactor = 1;
        } else if (scaleFactor == 1) {
            scaleFactor = 0.5;
        }
    }

    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");

    }

    public void init_loop() {

        buttonYPressed = 0;
        buttonAPressed = 0;
        buttonAPressedG2 = 0;
        robot.lift.setPower(0);
        robot.latch.setPower(0);
        robot.intake.setPower(0);
        robot.arm.setPower(0);
        robot.armExtrusion.setPower(0);
        robot.basket.setPower(0);
        robot.marker.setPosition(0);


    }

    public void loop() {

        /**
         *POV Mecanum Wheel Control With Strafing
         */

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = (r * Math.cos(robotAngle) + rightX) * scaleFactor * direction;
        final double v2 = (r * Math.sin(robotAngle) - rightX) * scaleFactor * direction;
        final double v3 = (r * Math.sin(robotAngle) + rightX) * scaleFactor * direction;
        final double v4 = (r * Math.cos(robotAngle) - rightX) * scaleFactor * direction;

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(v4);

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
        }
        else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.lift.setPower(-1);
        }
        else {
            robot.lift.setPower(0);
        }


        /**
         Latch Release Control
          */

        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.latch.setPower(-1);
        }
        else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.latch.setPower(1);
        }
        else {
            robot.latch.setPower(0);
        }

        /**
         *Intake Control
         */
        if (gamepad1.right_trigger > 0) {
            robot.intake.setPower(-0.50);
        }
        else if (gamepad1.right_bumper) {
            robot.intake.setPower(0.50);
        }
        else {
            robot.intake.setPower(0);
        }

        /**
         *Arm Extrusion
         */

        if (gamepad2.right_bumper) {
            robot.armExtrusion.setPower(-1);
        }
        else if (gamepad2.right_trigger > 0) {
            robot.armExtrusion.setPower(1);
        }
        else {
            robot.armExtrusion.setPower(0);
        }

        /**
        Arm Control
         */

        if (gamepad2.dpad_down && !gamepad2.dpad_up)        {
            robot.lift.setPower(1);
        }
        else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.lift.setPower(-1);
        }
        else {
            robot.lift.setPower(0);
        }

//        if (gamepad1.left_bumper) {
//            robot.arm.setPower(-1);
//        }
//        else if (gamepad1.left_trigger > 0) {
//            robot.arm.setPower(1);
//        }
//        else {
//            robot.arm.setPower(0);
//        }

        robot.arm.setPower(gamepad2.right_stick_y * -.75);



        /**
         * Basket
         */

        if (gamepad2.left_stick_y > 0) {
            robot.basket.setPower(0.5);
        }
        else if (gamepad2.left_stick_y < 0) {
            robot.basket.setPower(-0.5);
        }
        else {
            robot.basket.setPower(0);
        }

        /**
        * Telemetry
        */
        telemetry.addData("Scale Factor", scaleFactor);
        telemetry.addData("Direction", direction);
        telemetry.addData("left front power", robot.leftFront.getPower());
        telemetry.addData("left back power", robot.leftBack.getPower());
        telemetry.addData("right front power", robot.rightFront.getPower());
        telemetry.addData("right back power", robot.rightBack.getPower());
        telemetry.addData("Arm Encoder Ticks", robot.arm.getCurrentPosition());
        telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
        telemetry.addData("Arm Extrusion Encoder Ticks", robot.armExtrusion.getCurrentPosition());
        telemetry.update();
    }

    public void stop() {

        buttonYPressed = 0;
        buttonAPressed = 0;
        robot.lift.setPower(0);
        robot.latch.setPower(0);
        robot.intake.setPower(0);
        robot.arm.setPower(0);
        robot.armExtrusion.setPower(0);
        robot.basket.setPower(0);
        robot.marker.setPosition(0);

    }
}