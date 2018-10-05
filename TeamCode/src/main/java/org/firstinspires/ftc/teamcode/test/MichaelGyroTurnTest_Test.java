package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.MichaelGyroTurnTest;
import org.firstinspires.ftc.teamcode.MichaelGyroTurnTest2;
import org.firstinspires.ftc.teamcode.MichaelGyroTurnTest3;
import org.firstinspires.ftc.teamcode.MichaelRandomTesting;

public class MichaelGyroTurnTest_Test {

    /**
     * If you get a NullPointerException originating in the runOpMode() method like this:
     * <p>
     * java.lang.NullPointerException
     * at org.firstinspires.ftc.teamcode.MichaelGyroTurnTest2.runOpMode(MichaelGyroTurnTest2.java:34)
     * <p>
     * then you need to initialize the robot hardware in the class by
     * (1) adding/instantiating the HardwarePushBot class to your class and
     * (2) calling the init method on the HardwarePushBot class like this:
     * <p>
     * HardwarePushbot robot   = new HardwarePushbot();
     * robot.init(hardwareMap);
     * <p>
     * (3) Add the robot hardware configuration to the phone (left_drive, right_drive, etc.).
     * <p>
     * <p>
     * <p>
     * For the following 4 methods, if you get a NullPointerException like the one below:
     * <p>
     * java.lang.NullPointerException
     * at org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.init(HardwarePushbot.java:81)
     * <p>
     * This is correct.
     * <p>
     * The NullPointerException in this case is due to the hardware on the robot not
     * connected to this test class.  When you connect the robot to the phone and configure the
     * hardware (motors) on the phone, your program should run correctly.
     * <p>
     * <p>
     * Sean
     */

    MichaelGyroTurnTest michaelGyroTurnTest = new MichaelGyroTurnTest();
    MichaelGyroTurnTest2 michaelGyroTurnTest2 = new MichaelGyroTurnTest2();
    MichaelGyroTurnTest3 michaelGyroTurnTest3 = new MichaelGyroTurnTest3();
    MichaelRandomTesting michaelRandomTesting = new MichaelRandomTesting();

    public static void main(String[] args) {

        try {
            MichaelGyroTurnTest_Test obj = new MichaelGyroTurnTest_Test();
            obj.run(args);
        } catch (InterruptedException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    private void run(String[] args) throws Exception {

        michaelGyroTurnTest();
        michaelGyroTurnTest2();
        michaelGyroTurnTest3();
        michaelRandomTesting();
    }

    private void michaelGyroTurnTest() throws Exception {
        michaelGyroTurnTest.runOpMode();
    }

    private void michaelGyroTurnTest2() throws Exception {
        michaelGyroTurnTest2.runOpMode();
    }

    private void michaelGyroTurnTest3() throws Exception {
        michaelGyroTurnTest3.runOpMode();
    }

    private void michaelRandomTesting() throws Exception {
        michaelRandomTesting.runOpMode();
    }

}
