package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Strafing Test", group="Pushbot")
public class StrafingTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMapStrafingTest robot = new HardwareMapStrafingTest();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;


        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.leftFront.setPower(left);
            robot.leftBack.setPower(left);
            robot.rightFront.setPower(right);
            robot.rightBack.setPower(right);

            /**
             Strafing on left stick x axis
             */

            if (gamepad1.left_stick_x < 0) {
                robot.leftFront.setPower(left);
                robot.leftBack.setPower(-left);
                robot.rightFront.setPower(-right);
                robot.rightBack.setPower(right);
            } else if (gamepad1.left_stick_x > 0) {
                robot.leftFront.setPower(-left);
                robot.leftBack.setPower(left);
                robot.rightFront.setPower(right);
                robot.rightBack.setPower(-right);
            }
        }
    }
}
