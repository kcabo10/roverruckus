package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class GridNavAndGryo {

    GridNavigationImplimentation gridnav = new GridNavigationImplimentation();
    MichaelGyroTurnTest4 gyro = new MichaelGyroTurnTest4();

    double xOrigin = 0;
    //X1 is starting X coordinate
    double xDestination;
    //X2 is X destination
    double yOrigin= 0;
    //Y1 is starting Y coordinate
    double yDestination;
    //Y2 is Y destination
    double StartingAngle = 0;
    //X1 is starting X coordinate
    //Y1 is starting Y coordinate
    double Distance;
    double turnAngle = 0;


    public void GridNavImplimented(double xDestination, double yDestination, double power){
        gridnav.GridNavDriveDistance(xDestination, yDestination);
        gridnav.GridNavTurnAngle(xDestination, yDestination);

        gyro.turnGyro((float) turnAngle);

        double COUNTS_PER_INCH = 1120;
        gyro.LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gyro.LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro.LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro.RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gyro.RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gyro.LeftFront.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        gyro.LeftBack.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        gyro.RightFront.setTargetPosition((int)(Distance * COUNTS_PER_INCH));
        gyro.RightBack.setTargetPosition((int)(Distance * COUNTS_PER_INCH));

        gyro.LeftFront.setPower(power);
        gyro.LeftBack.setPower(power);
        gyro.RightFront.setPower(power);
        gyro.RightBack.setPower(power);

        while((gyro.LeftFront.getCurrentPosition() < (Distance * COUNTS_PER_INCH))
                && (gyro.RightFront.getCurrentPosition() < (Distance * COUNTS_PER_INCH))){}

        gyro.LeftFront.setPower(0);
        gyro.LeftBack.setPower(0);
        gyro.RightFront.setPower(0);
        gyro.RightBack.setPower(0);

        gyro.LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gyro.RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}