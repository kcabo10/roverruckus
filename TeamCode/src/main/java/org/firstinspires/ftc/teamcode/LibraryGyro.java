package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous(name="Michael Gryo Testing 4", group="Exercises")
//@Disabled

public class LibraryGyro extends LinearOpMode {
    DcMotor LeftFront;
    DcMotor LeftBack;
    DcMotor RightFront;
    DcMotor RightBack;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double angle_variable;
    double speed;
    boolean aButton, bButton, touched;


    long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;

    public LibraryGyro(){

        LeftFront = hardwareMap.dcMotor.get("left_front");
        LeftBack = hardwareMap.dcMotor.get("left_back");

        RightFront = hardwareMap.dcMotor.get("right_front");
        RightBack = hardwareMap.dcMotor.get("right_back");
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        // get a reference to REV Touch sensor.
//        touch = hardwareMap.digitalChannel.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
    }

    // called when init button is  pressed.
//    @Override
    public void runOpMode() throws InterruptedException {
//
//        LeftFront = hardwareMap.dcMotor.get("left_front");
//        LeftBack = hardwareMap.dcMotor.get("left_back");
//
//        RightFront = hardwareMap.dcMotor.get("right_front");
//        RightBack = hardwareMap.dcMotor.get("right_back");
//        RightFront.setDirection(DcMotor.Direction.REVERSE);
//        RightBack.setDirection(DcMotor.Direction.REVERSE);
//
//        // get a reference to REV Touch sensor.
////        touch = hardwareMap.digitalChannel.get("touch_sensor");
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);
//
//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !imu.isGyroCalibrated()) {
//            sleep(50);
//            idle();
//        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        turnGyro(90);

        sleep(1000);

        turnGyro(-90);

        while (opModeIsActive()) {

            getAngle();
            angle_variable = getAngle();
            telemetry.addData("Mode", "running");
            telemetry.addData("Current Angle", getAngle());
            telemetry.update();

            // drive until end of period.
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
        updateTelemetry(telemetry);
//        sleep(5000);

        //Input = currentHeading;

        do {

            ComputePID();
            LeftFront.setPower(-Output);
            LeftBack.setPower(-Output);
            RightFront.setPower(Output);
            RightBack.setPower(Output);
            timer++;
            //sleep(1000);
            Input = getAngle();
            //sleep(1000);
            telemetry.addData("curHeading", Input);
            telemetry.addData("tarHeading", Setpoint);
            updateTelemetry(telemetry);
            //} while (Input < targetHeading && (System.currentTimeMillis() < (startTime + 6000)));
        } while ((Math.abs(Input - Setpoint) > TOLERANCE) || (System.currentTimeMillis() < (startTime + 3000)));


        telemetry.addData("curHeading", Input);
        telemetry.addData("tarHeading", Setpoint);
        telemetry.addData("leftPwr", -Output);
        telemetry.addData("rightPwr", Output);
        //telemetry.addData("headingErr", headingError);
        //telemetry.addData("driveSteer", driveSteering);
        telemetry.addData("DRIVEGAIN", DRIVEGAIN);
        updateTelemetry(telemetry);


        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        sleep(10000);

        return Input;
    }

}