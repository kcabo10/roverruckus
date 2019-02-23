package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

/**
 * Created by kyliestruth 10/5/17.
 */

@TeleOp(name= "TeleOp Program", group= "TankDrive" )
public class TeleOpProgram extends OpMode
{
    private HardwareBeep robot = new HardwareBeep();

    private int buttonYPressed;
    private int buttonAPressed;
    private boolean manual_mode = false;
    private int direction = -1;
    private double scaleFactor = 1;
    private double scaleTurningSpeed = 1;
    int arm_state = 0;
    int arm_extrusion_state = 0;
    int basket_state = 0;
    public ElapsedTime armtime = new ElapsedTime();
    public ElapsedTime basketTime = new ElapsedTime();
    // 0 = waiting, 1 = arm up commanded, 2 = arm down commanded

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
        robot.lift.setPower(0);
        robot.latch.setPower(0);
        robot.intake.setPower(0);
        robot.arm.setPower(0);
        robot.armExtrusion.setPower(0);

    }

    public void loop() {

        robot.colorSensor.enableLed(true);

        /**
         *POV Mecanum Wheel Control With Strafing
         */

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        if (direction == -1) {
            final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);

        } else {
            final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
            final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;

            robot.leftFront.setPower(v1);
            robot.rightFront.setPower(v2);
            robot.leftBack.setPower(v3);
            robot.rightBack.setPower(v4);
        }



        /**
         *Invert Direction On Y Button
         */

        switch (buttonYPressed) {
            case (0):
                if (gamepad1.y) {
                    buttonYPressed = 1;
                }
                break;
            case (1):
                if (!gamepad1.y) {
                    reverseDirection();
                    buttonYPressed = 0;
                }
                break;
        }

        /**
         *ScaleFactor on A Button
         */


        switch (buttonAPressed) {
            case (0):
                if (gamepad1.a) {
                    buttonAPressed = 1;
                }
                break;
            case (1):
                if (!gamepad1.a) {
                    buttonAPressed = 0;
                    scaleFactor();
                }
                break;
        }

        /**
         * Scale Turning on Right Stick Button
         */

        if (gamepad1.right_stick_button) {
            scaleTurningSpeed = 0.5;
        } else {
            scaleTurningSpeed = 1;
        }

        /**
         *Lift Control
         */


        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            robot.lift.setPower(1);
        } else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.lift.setPower(-1);
        } else {
            robot.lift.setPower(0);
        }

        /**
         * Latching
         */

        if (gamepad2.dpad_right) {
            robot.latch.setPower(-1);
        } else if (gamepad2.dpad_left) {
            robot.latch.setPower(1);
        } else robot.latch.setPower(0);

        /**
         *Intake Control
         */
        if (gamepad1.right_trigger > 0) {
            robot.intake.setPower(-0.50);
        } else if (gamepad1.right_bumper) {
            robot.intake.setPower(0.50);
        } else {
            robot.intake.setPower(0);
        }

        /**
         *Arm Extrusion
         */

        switch (arm_extrusion_state) {
            case 0:
                if (gamepad2.right_bumper && robot.touchSensor.getState()) {
                    robot.armExtrusion.setPower(1);
                    robot.basket.setPosition(.5);
                    arm_extrusion_state++; //moving
                } else if (gamepad2.right_trigger > 0) {
                    robot.armExtrusion.setPower(-1);
                    robot.basket.setPosition(.3);
                } else {
                    robot.armExtrusion.setPower(0);
                }
                break;
            case 1:
                if (!gamepad2.right_bumper) {
                 arm_extrusion_state++;
                }
                break;
            case 2:
                // Last state before it goes back to state 0. This state has a timer to ensure that the motor stops at 2 seconds.
                if (!robot.touchSensor.getState()) {
                    robot.armExtrusion.setPower(0);
                    arm_extrusion_state = 0;
                }
                else if (gamepad2.right_bumper) {
                    arm_extrusion_state++;
                }
                break;
            case 3:
                if (!gamepad2.right_bumper) {
                    robot.armExtrusion.setPower(0);
                    arm_extrusion_state = 0;
                }
                break;
        }

        /**
         Arm Control
         */

        //int arm_state;
        // 0 = waiting for command, 1 = commanded, 2 = waiting for arm.isbusy==false
        switch (arm_state) {
            case 0:
                // This state is the constant state that waits for the trigger/bumper/slide to be pressed/pushed

                robot.arm.setPower(gamepad2.right_stick_y * .75);

                if (gamepad1.left_bumper) {
                    // Moving arm down
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(500);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.arm.setPower(.75);
                    arm_state = 1;

                } else if (gamepad1.left_trigger > 0) {
                    // Moving arm up
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-580);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.arm.setPower(1);
                    arm_state = 1;
                }
                break;
            case 1:
                // Once it recognizes that the controller has been moved, and the power is set, then it initializes this next state.
                armtime.reset();
                if (robot.arm.isBusy()) {
                    arm_state = 2; //moving
                }
                break;
            case 2:
                // Last state before it goes back to state 0. This state has a timer to ensure that the motor stops at 2 seconds.
                if (!robot.arm.isBusy() || armtime.seconds() >= 2) {
                    arm_state = 0;
                    robot.arm.setPower(0);
                }
                break;

        }

        // MANUAL OVERRIDE
        if ((gamepad2.right_stick_y > .05 && gamepad2.right_stick_y <= 1) ||
                (gamepad2.right_stick_y < -.05 && gamepad2.right_stick_y >= -1)) {
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm_state = 0;
        }

        /**
         Basket
         */
        // check to see if we need to move the servo.
        if (gamepad2.a) {
            // move to 0 degrees.
            robot.basket.setPosition(0);
            telemetry.addData("Button a pressed", gamepad2.a);
            telemetry.update();
        } else if (gamepad2.x) {
            robot.basket.setPosition(.5);
            telemetry.addData("Button x pressed", gamepad2.x);
            telemetry.update();
        } else if (gamepad2.b) {
            robot.basket.setPosition(.9);
        }
        switch (basket_state) {
            case (0):
                if (gamepad2.y) {
                    basketTime.reset();
                    robot.basket.setPosition(.25);
                    basket_state++;
                }
            break;
            case (1):
                if (robot.basket.getPosition() >= .25 && basketTime.seconds() > .25) {
                    robot.basket.setPosition(0);
                    basket_state = 0;
                }
            break;
        }


        /**
        * Telemetry
        */

        telemetry.addData("y Button", buttonYPressed);
        telemetry.addData("Lift Encoder Ticks", robot.lift.getCurrentPosition());
        telemetry.addData("Color Number", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
        telemetry.addData("arm_state", arm_state);
        telemetry.addData("arm_extrusion_state", arm_extrusion_state);
        telemetry.addData("touch sensor", robot.touchSensor.getState());
        telemetry.addData("manual_mode", manual_mode);
        telemetry.addData("Scale Factor", scaleFactor);
        telemetry.addData("Direction", direction);
        telemetry.addData("left front power", robot.leftFront.getPower());
        telemetry.addData("left back power", robot.leftBack.getPower());
        telemetry.addData("right front power", robot.rightFront.getPower());
        telemetry.addData("right back power", robot.rightBack.getPower());
        telemetry.addData("Arm Encoder Ticks", robot.arm.getCurrentPosition());
        telemetry.addData("Arm Extrusion Encoder Ticks", robot.armExtrusion.getCurrentPosition());
        telemetry.addData("Arm Extrusion Power", robot.armExtrusion.getPower());
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

    }
}