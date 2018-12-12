package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;


@I2cSensor(name = "SensorMB1242", description = "Ultrasonic Sensor from MaxBotix", xmlTag = "MB1242")
public class SensorMB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "MaxBotix MB1242 Ultrasonic Sensor";
    }

    public SensorMB1242(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(224));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}