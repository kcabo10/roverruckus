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
    boolean manual_mode = false;
    boolean lift_mode = false;
    boolean lower_mode = false;
    private int direction = -1;
    private double scaleFactor = 1;
    int arm_state = 0;
    int auto_lift = 0;
    int auto_lower = 0;
    public ElapsedTime autolifttime = new ElapsedTime();
    public ElapsedTime colorsensortime = new ElapsedTime();
    public ElapsedTime armtime = new ElapsedTime();

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
        robot.basket.setPower(0);
        robot.marker.setPosition(0);

    }

    public void loop() {

        robot.colorSensor.enableLed(true);

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


//        if (gamepad2.dpad_down && !gamepad2.dpad_up)        {
//            robot.lift.setPower(1);
//        }
//        else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
//            robot.lift.setPower(-1);
//        }
//        else {
//            robot.lift.setPower(0);
//        }


        /**
         Latch Release Control
          */
//        if (gamepad2.dpad_right) {
//            colorSensorTime.reset();
//            while ((robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != 3) && (colorSensorTime.seconds() < 1)) {
//                telemetry.addData("Color Number", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
//                telemetry.update();
//                robot.latch.setPower(-1);
//            }
//        }
//        else if (gamepad2.dpad_left) {
//            colorSensorTime.reset();
//            while ((robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != 10) && (colorSensorTime.seconds() < 1)) {
//                telemetry.addData("Color Number", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
//                telemetry.update();
//                robot.latch.setPower(1);
//            }
//        }
//        else robot.latch.setPower(0);
        /**
         * Auto Lift and Latch With an Override
         */

        /**
         * if !gamepad.y
         *    autolifttime.reset()
         * If gamepad y && !autoLiftInProgress
         *    start autolift
         *    start timer
         *    autoLiftInProgress = TRUE;
         * If timer > 3000 && !autoLiftInProgress
         *    stop autolift
         *    go manual
         *    autliftInProgress == FALSE

         */


        /**
         * case
         * WAITING FOR AUTOLIFT 0
         * START LATCH 1
         * AUTOLIFT IN PROGRESS 1
         * AUTOLIFT WAIT FOR COLOR SENSOR 2
         * AUTOLIFT WAIT FOR LIFT 3
         * AUTOLIFT DONE 4
         * MANUAL MODE 5
         */

        if (!manual_mode && !lower_mode) {
            lift_mode = true;
            switch (auto_lift) {
                case 0:
                    //WAIT FOR AUTO_LIFT BUTTON
                    if (gamepad2.dpad_up) {
                        autolifttime.reset();
                        auto_lift++;
                    }
                    break;
                case 1:
                    //START LATCH
                    robot.latch.setPower(-1);
                    auto_lift++;
                    break;
                case 2:
                    //CHECK COLOR AND START LIFT
                    if (colorsensortime.milliseconds() > 200) {
                        if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 3) {
                            robot.latch.setPower(0);
                            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            robot.lift.setTargetPosition(-16000);
                            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.lift.setPower(1);
                            auto_lift++;
                        }
                        colorsensortime.reset();
                    }
                    break;
            }
        }
        if(!manual_mode && !lift_mode) {
            lower_mode = true;
            switch (auto_lower) {
                case 0:
                    //WAIT FOR AUTO_LIFT BUTTON
                    if (gamepad2.dpad_down) {
                        autolifttime.reset();
                        auto_lower++;
                    }
                    break;
                case 1:
                    //START LATCH
                    robot.latch.setPower(1);
                    auto_lower++;
                    break;
                case 2:
                    //CHECK COLOR AND START LATCH
                    if (colorsensortime.milliseconds() > 200) {
                        if (robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 10) {
                            robot.latch.setPower(0);
                            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            robot.lift.setTargetPosition(16000);
                            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.lift.setPower(1);
                            auto_lower++;
                        }
                        colorsensortime.reset();
                    }
                    break;
            }
        }


    if (autolifttime.seconds() > 3 && gamepad2.dpad_up && auto_lift > 0) {
         manual_mode = true;
    }

    if (manual_mode) {
        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            robot.lift.setPower(1);
        } else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.lift.setPower(-1);
        } else {
            robot.lift.setPower(0);
        }


        if (gamepad2.dpad_right) {
                robot.latch.setPower(-1);
        }
        else if (gamepad2.dpad_left) {
                robot.latch.setPower(1);
        }
        else robot.latch.setPower(0);
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

        //int arm_state;
        // 0 = waiting for command, 1 = commanded, 2 = waiting for arm.isbusy==false
        switch (arm_state)
        {
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
                }
                else if (gamepad1.left_trigger > 0) {
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
                if (robot.arm.isBusy()){
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
                (gamepad2.right_stick_y < -.05 && gamepad2.right_stick_y >= -1)){
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm_state = 0;
        }

        /**
         Basket
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

        telemetry.addData("Color Number", robot.colorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
        telemetry.addData("arm_state", arm_state);
        telemetry.addData("auto_lift", auto_lift);
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