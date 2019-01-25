package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareBeep;
import org.firstinspires.ftc.teamcode.sensors.SensorMB1242;

import org.firstinspires.ftc.teamcode.sensors.SensorMB1242;

@TeleOp(name="Rear Ultrasonic Testing", group="Test")
//@Disabled
public class MB1242_Test extends LinearOpMode {
    HardwareBeep robot = new HardwareBeep();

    SensorMB1242 rearUS;
    boolean last = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

//        rearUS = hardwareMap.get(SensorMB1242.class,"rear");
        rearUS = robot.ultrasonic;

//        rearUS.init(telemetry);

        rearUS.startAutoPing(40);
        sleep(2000);
        telemetry.addData("Start AutoPing", "");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("waitForStart()", "");
        telemetry.update();
        sleep(2000);
//        rearUS.run();
//        telemetry.addData("rearUS.run", "");
//        telemetry.update();
//        sleep(2000);
        telemetry.addData("Should have read sensor", "");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&!isStopRequested()) {
            rearUS.run();

            telemetry.addData("Distance",rearUS.getDistance());
            telemetry.update();
        }

        rearUS.close();
    }
}