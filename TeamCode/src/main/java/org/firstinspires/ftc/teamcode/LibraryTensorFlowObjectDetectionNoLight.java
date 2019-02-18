/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.LinkedList;
import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
public class LibraryTensorFlowObjectDetectionNoLight {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /**
     * Set Vuforia Key so it knows what phone it is connecting to
     */
    //private static final String VUFORIA_KEY = "AehWUEP/////AAAAGdLM1Ir3CEUunWFOGlSVegZ02oYjauBrfpYGcP/MNvZGEWO15KaOdjuIx0XAGISDJtiT9pfALwG5bGHfY2d5LVLV3jBq+2vLfcYh7zxUbHOcJpPfbzpUDVkGI5WHZlZ6IaqoCAEPznkxcZ5uyMwfZr1qyZp9LVTTAFhYwjRgSuF4/mcjzI3/ujUOZEKUzIOQbSlAPyNkiNMnRA0RHlzK7djpkXvghYsX7LYJDnJc5Fvpi6mqZqI+lyco0jnUHhMh4l7HczZ1HbKTAwuJFqc3aQab8bnjw9QegJb62vURA/ljwEIEUhT6mEGx+XJSOUA+KCwi/WDnKcZwOZr43VqmHPgLCvJmTFpVeOdBY4ozX5/J";
    //private static final String VUFORIA_KEY = "AS9xNSf/////AAABmZ2AubQkTEFHuWxboHr/qgcSu4jWVAzmohkEVAicGMw66GcX1uY55PbVhytIM4mQxb4WUjuz3eIg3QzfmZ1a9jPPREvj90fYcxhBmk6+444WAselalZu6Kw/xDM7ibuO3Md5STFLGzKUsZDLajzqjeZT67DuCxWJDCUutKZoVbmOS7kgNEbuHNO9LJuq80OTL7aiNsAjqLtAdduHT8nqSCz4wjQ1pbsZ1Ds809JN0PHu3nHC+dbWj8qqKUiFkEo0Z38g1tanehxI8vvJ+Rj/ezymKCsUeXhkgZ7CDq/uwAitEi75qpQZv1fs4ctcLoZVvpn4oGrmpv1QItYOsPa7E79ed5izdd021d5Z1RUya4bh";

    private static final String VUFORIA_KEY = "Aa4mtdP/////AAABmSRcR7UP9kS4nIeX1am8Tf5TlWuaSoXF9p9tlyFSx0zDxT39pe+kg1dseqSvlAQBMws92KngQN7wl3RHkCgjre8b+A9RXXtGx0mlQ1PWbMIf4AlDdHncv6ERajxzi+HwOgFkMt44eQ9gVLBLUvxzDepzfZaMSfalcWz3qtbhq8hH2R3npGb+p2x6XVY6IWZSwkKpnCFVddAhsyuToQ/S5ndIkeB2O4mquvWESjFDc6ALl/SU7Rcg5Qb/chtv2dK+EWkcaf+XSjzn7KvOsaykUeOk2ChCIEQizneBH0ILH28lPMGjxTky7qnTf+5Jb/IHpd64ZtTZN9Q2Nyrlce1750yUVtnqSxRdUPPaJTiBQrKo";

    /**
     * Set Vuforia as a Localizer
     */
    private VuforiaLocalizer vuforia;

    /**
     * Set tfod to the TensorFlowObjectDetector
     */
    private TFObjectDetector tfod;
    HardwareBeep robot;
    Telemetry telemetry;

    public LibraryTensorFlowObjectDetectionNoLight(HardwareBeep newHardwareBeep, Telemetry newTelemetry) {

        robot = newHardwareBeep;

        telemetry = newTelemetry;

    }

    public String findMineral() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        /**
         * Turn on the light on phone to make mineral visible
         */
        phoneLight(false);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
        long startTime = 0;
        String previousPosition = "";
        String goldPosition = "";

        /**
         * Sets the TensorFlow to read the mineral for at least 3 seconds to verify that it is the correct mineral
         */

        startTime = System.currentTimeMillis();


        while (System.currentTimeMillis() < (startTime + 3000)) {

            goldPosition = readMineral();

            if (goldPosition == previousPosition) {
            } else {
                previousPosition = goldPosition;
                startTime = System.currentTimeMillis();

            }
            telemetry.addData("StartTime: ", startTime);
            telemetry.addData("CurrentTime: ", System.currentTimeMillis());
            telemetry.addData("Prev Position:  ", previousPosition);
            telemetry.addData("Gold Position:  ", goldPosition);
            telemetry.update();
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        telemetry.addData("Mineral Position: ", goldPosition);
        telemetry.update();
        phoneLight(false);
        return goldPosition;
    }

    /**
     * Begin reading the mineral
     */
    public String readMineral() {
        String currentPos = "";
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (currentPos == "" && timer.seconds() < 6) {
            if (tfod != null) {                    // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() >= 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        LinkedList<Recognition> recognitionLinkedList = new LinkedList<Recognition>();
                        telemetry.addData("New Linked List creation", "");
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData("Iterate over updatedRecognitions", updatedRecognitions.indexOf(recognition));
                            telemetry.update();
                            /**
                             * Set the camera to read two minerals and decide what position gold is in
                             */

                            if ((recognitionLinkedList.isEmpty()) ||
                                    (recognition.getBottom() > recognitionLinkedList.getFirst().getBottom())) {
                                //telemetry.addData("Added element to recognitionLinkedList", "");
                                //sleep(2000);
                                recognitionLinkedList.addFirst(recognition);
                            } else {
                                recognitionLinkedList.add(recognition);
                            }
                        }
                        Recognition recognition = null;
                        for (int i = 0; i < 2; i++) {
                            //telemetry.addData("iterate over first two elements: element ", i);
                            //telemetry.addData("List size", recognitionLinkedList.size());
                            //telemetry.update();
                            recognition = recognitionLinkedList.get(i);
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        /**
                         * Choose which position is correct
                         */
                        if (goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            currentPos = "LEFT";
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1) {
                            if (goldMineralX > silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                currentPos = "RIGHT";
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                currentPos = "CENTER";
                            }
                        }
                    }
                }
            }
        }
        return currentPos;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void phoneLight(boolean on) {

        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(on);

    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
// Quinn was here
