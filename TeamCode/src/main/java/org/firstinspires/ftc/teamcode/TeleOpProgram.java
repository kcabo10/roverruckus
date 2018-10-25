package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by kyliestruth 10/5/17.
 */

@TeleOp(name= "TeleOp Program", group= "TankDrive")
public class TeleOpProgram extends OpMode
{
    private HardwareBeep robot = new HardwareBeep();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");

    }

    public void init_loop() {
        robot.lift.setPower(0);
    }

    public void loop() {

        double frontLeftY = gamepad1.left_stick_y;
        double frontRightY = gamepad1.right_stick_y;
        double backLeftY = gamepad1.left_stick_y;
        double backRightY = gamepad1.right_stick_y;

        double frontLeftX = gamepad1.left_stick_x;
        double frontRightX = gamepad1.right_stick_x;
        double backLeftX = gamepad1.left_stick_x;
        double backRightX = gamepad1.right_stick_x;

        robot.rightFront.setPower((/*deadZoneY */ frontRightY) + (/*deadZoneX */ frontRightX));
        robot.rightBack.setPower((/*deadZoneY */ backRightY) - (/*deadZoneX */ backRightX));
        robot.leftFront.setPower((/*deadZoneY */ frontLeftY) - (/*deadZoneX */ frontLeftX));
        robot.leftBack.setPower((/*deadZoneY */ backLeftY) + (/*deadZoneX */ backLeftX));


        /*
        Intake Control
         */

        if (gamepad2.right_trigger > 0) {
            robot.lift.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.lift.setPower(1);
            robot.lift.setTargetPosition(1);

            robot.lift.setPower(1);
        } else if (gamepad2.right_trigger > 0) {
            robot.lift.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.lift.setPower(-1);
            robot.lift.setTargetPosition(0);
        }
    }
    
    public void stop() {

        robot.lift.setPower(0);

    }
}
