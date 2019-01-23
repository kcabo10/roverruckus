package org.firstinspires.ftc.teamcode;


import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

@Autonomous(name="Testing Ultrasonic Sensor", group="Exercises")
public class TestingMB1242Sensor extends LinearOpMode {

    public HardwareBeep robot   = new HardwareBeep();

//    Parameters

//    address: the 7-bit I2C device address of the device to transmit to and from.
//    quantity: the number of bytes to request.
//    value: a value to send as a single byte.

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        robot.init(hardwareMap);


        waitForStart();

    }
}