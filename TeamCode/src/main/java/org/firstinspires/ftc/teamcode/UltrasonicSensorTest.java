package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class UltrasonicSensorTest extends LinearOpMode {

    HardwareBeep robot = new HardwareBeep();
    HardwareMap hwMap = null;
    Telemetry telemetry = null;
    private DistanceSensor sensorRange;
    public double gridPos[] = new double[1];

    public double Distance = 0;

    @Override
    public void runOpMode() {


        // you can use this as a regular DistanceSensor.
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");


        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", sensorRange.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            Distance = sensorRange.getDistance(DistanceUnit.INCH);
            gridPos[0] = Distance / 24;
            telemetry.addData("GridNav Pos", "{X} = %.1f", gridPos[0]);
            telemetry.update();
        }
    }
}
