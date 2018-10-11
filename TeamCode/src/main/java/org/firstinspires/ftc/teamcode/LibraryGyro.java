package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous(name="Michael Gryo Testing 4", group="Exercises")
//@Disabled

public class LibraryGyro {

    HardwareBeep robot = null;
    Telemetry telemetry;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double angle_variable;
    double speed;
    boolean aButton, bButton, touched;

    long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;


    /**
     * The hardware class needs to be initialized before this function is called
    */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry){
        robot = myRobot;
        telemetry = myTelemetry;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    public void ComputePID() {
        long now = System.currentTimeMillis();
        double timeChange = (double) (now - lastTime);
        double error = Setpoint - Input;
        errSum += (error * timeChange);
        double dErr = (error - lastErr);

        Output = kp * error + ki * errSum + kd * dErr;
        lastErr = error;
        lastTime = now;
    }

    public void SetTunings (double Kp, double Ki, double Kd)
    {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }



    public double turnGyro(float targetHeading) {
        int original_anglez = 0;
//        BNO055IMU imu;
        int xVal, yVal, zVal = 0;
        int heading = 0;
        int angleZ = 0;
        float MIDPOWER = 0;
        double DRIVEGAIN = 1;
        double TOLERANCE = .5;
        int timer = 0;
        double currentHeading, headingError, driveSteering, leftPower, rightPower, oldCurrentHeading = 0.0;
        long startTime = 0;
//        imu = (BNO055IMU) hardwareMap.gyroSensor.get("imu");
        resetAngle();
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        telemetry.addData("Current Pos", currentHeading);
//        updateTelemetry(telemetry);

        startTime = System.currentTimeMillis();
        currentHeading = getAngle();
        SetTunings(.01, 0, 0.1);

        Setpoint = targetHeading;
        Input = getAngle();
        telemetry.addData("Current Pos ", currentHeading);
        telemetry.addData("Setpoint ", Setpoint);
        telemetry.addData("Input ", Input);
        telemetry.update();
        //        sleep(5000);

        //Input = currentHeading;

        do {

            ComputePID();
            robot.leftFront.setPower(-Output);
            robot.leftBack.setPower(-Output);
            robot.rightFront.setPower(Output);
            robot.rightBack.setPower(Output);
            timer++;
            //sleep(1000);
            Input = getAngle();
            //sleep(1000);
            telemetry.addData("curHeading", Input);
            telemetry.addData("tarHeading", Setpoint);
            telemetry.update();
            //} while (Input < targetHeading && (System.currentTimeMillis() < (startTime + 6000)));
        } while ((Math.abs(Input - Setpoint) > TOLERANCE) || (System.currentTimeMillis() < (startTime + 3000)));


        telemetry.addData("curHeading", Input);
        telemetry.addData("tarHeading", Setpoint);
        telemetry.addData("leftPwr", -Output);
        telemetry.addData("rightPwr", Output);
        //telemetry.addData("headingErr", headingError);
        //telemetry.addData("driveSteer", driveSteering);
        telemetry.addData("DRIVEGAIN", DRIVEGAIN);
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        sleep(10000);

        return Input;
    }

}