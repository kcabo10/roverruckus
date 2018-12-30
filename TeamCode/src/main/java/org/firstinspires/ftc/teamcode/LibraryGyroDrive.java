package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous(name="Michael Gryo Testing 4", group="Exercises")
//@Disabled

public class LibraryGyroDrive {

    HardwareBeep robot = new HardwareBeep();

    Telemetry telemetry;
    LibraryPIDController pidRotate, pidDrive;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .05, correction;
    double angle_variable;
    double speed;
    boolean aButton, bButton, touched;

    long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;

    DcMotor motor;


    /**
     * The hardware class needs to be initialized before this f unction is called
     */
    public void init(HardwareBeep myRobot, Telemetry myTelemetry, DcMotor myMotor) {
        robot = myRobot;
        telemetry = myTelemetry;
        motor = myMotor;

    }

    public void driveGyro(double power, int targetEncoderTicks) {

        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to REV Touch sensor.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        //imu = robot.hwMap.get(BNO055IMU.class, "imu");

        robot.imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new LibraryPIDController(.005, 0, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new LibraryPIDController(.05, 0, 0);

        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setTargetPosition(targetEncoderTicks);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // drive until end of period.

        do {

            // Use PID with imu input to drive in a straight line.
            correction = pidDrive.performPID(getAngle());

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 commanded power", power);

            // set power levels.
            robot.leftFront.setPower(-power + correction);
            robot.leftBack.setPower(-power + correction);

            robot.rightFront.setPower(-power - correction);
            robot.rightBack.setPower(-power - correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.
            telemetry.addData("left front power", robot.leftFront.getPower());
            telemetry.addData("left back power", robot.leftBack.getPower());
            telemetry.addData("right front power", robot.rightFront.getPower());
            telemetry.addData("right back power", robot.rightBack.getPower());
            telemetry.addData("right front ticks", robot.rightFront.getCurrentPosition());
            telemetry.update();
        } while (robot.rightFront.isBusy());
        //} while (robot.rightFront.getCurrentPosition() < targetEncoderTicks);

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);

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
        correction = checkDirection();

        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    private void setMotorPower(double powerL, double powerR) {

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftBack.setPower(powerL);
        robot.leftFront.setPower(powerL);
        robot.rightBack.setPower(powerR);
        robot.rightFront.setPower(powerR);
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
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        telemetry.addData("Current Pos", currentHeading);
//        updateTelemetry(telemetry);

        startTime = System.currentTimeMillis();
        currentHeading = getAngle();
        SetTunings(.02, 0, 0.1);

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
            robot.leftFront.setPower(Output);
            robot.leftBack.setPower(Output);
            robot.rightFront.setPower(-Output);
            robot.rightBack.setPower(-Output);
            timer++;
            //sleep(1000);
            Input = getAngle();
            //sleep(1000);
            telemetry.addData("curHeading", Input);
            telemetry.addData("tarHeading", Setpoint);
            telemetry.update();
            //} while (Input < targetHeading && (System.currentTimeMillis() < (startTime + 6000)));
        }
        while ((Math.abs(Input - Setpoint) > TOLERANCE) || (System.currentTimeMillis() < (startTime + 1050)));


        telemetry.addData("curHeading", Input);
        telemetry.addData("tarHeading", Setpoint);
        telemetry.addData("leftPwr", -Output);
        telemetry.addData("rightPwr", Output);
        //telemetry.addData("headingErr", headingError);
        //telemetry.addData("driveSteer", driveSteering);
        telemetry.addData("DRIVEGAIN", DRIVEGAIN);
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        return Input;


    }
}