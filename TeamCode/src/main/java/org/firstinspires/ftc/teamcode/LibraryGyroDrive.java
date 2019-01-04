package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class LibraryGyroDrive {

    HardwareBeep robot = new HardwareBeep();
    LibraryGyro   gyro    = new LibraryGyro();                    // Additional Gyro device
    Telemetry telemetry;
    DcMotor motor;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable


    /**
     * The hardware class needs to be initialized before this f unction is called
     */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry, DcMotor myMotor) {
        robot = myRobot;
        telemetry = myTelemetry;
        gyro.init(robot, telemetry);
        motor = myMotor;

    }

    public void gyroDrive ( double speed,
                            int encoderTicks,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        telemetry.addData("In Gyro Drive method", "");
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newLeftTarget = encoderTicks;
            newLeftTarget = encoderTicks;
            newRightTarget = encoderTicks;
            newRightTarget = encoderTicks;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFront.setTargetPosition(newLeftTarget);
            robot.leftBack.setTargetPosition(newLeftTarget);
            robot.rightFront.setTargetPosition(newRightTarget);
            robot.rightBack.setTargetPosition(newRightTarget);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            telemetry.addData("Code pos 1", "");
//            telemetry.update();
//            sleep(2000);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, .3);
            robot.leftFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.rightBack.setPower(speed);

//            telemetry.addData("Code pos 2", "");
//            telemetry.update();
//            sleep(2000);


            // keep looping while we are still active, and BOTH motors are running.
            while (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (encoderTicks < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                leftSpeed = Range.clip(leftSpeed, -.3, .3);
                rightSpeed = Range.clip(rightSpeed, -.3, .3);

                robot.leftFront.setPower(leftSpeed);
                robot.leftBack.setPower(leftSpeed);
                robot.rightFront.setPower(rightSpeed);
                robot.rightBack.setPower(rightSpeed);

//                telemetry.addData("Code pos 4", "");
//                telemetry.update();



                // Display drive status for the driver.
                telemetry.addData("Error" ,  error);
                telemetry.addData("Steer", steer);
//                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftFront.getCurrentPosition(),
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftBack.getCurrentPosition(),
//                                                             robot.rightFront.getCurrentPosition()));
//                                                             robot.rightBack.getCurrentPosition();
                telemetry.addData("L Speed", leftSpeed);
                telemetry.addData("R Speed", rightSpeed);
                telemetry.update();

            }

//            telemetry.addData("GyroDrive, after drive commands", "");
//            telemetry.update();
//            sleep(2000);

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }



    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;


    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}