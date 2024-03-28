/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "RRRedLeft", group = "Concept")
//@Disabled
public class RRRedLeft extends LinearOpMode {

    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor rLift = null;
    private DcMotor lLift = null;
    //private DcMotor vector = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private CRServo dread = null;
    private Servo leftPull = null;
    private Servo rightPull = null;
    private Servo outtake = null;
    private CRServo intakein = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "red_flex.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "red_flex",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        //hardware mapping
        leftFront = hardwareMap.get(DcMotor.class,"leftFront"); //frontleft, port 0
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");  //frontright, port 1
        leftRear = hardwareMap.get(DcMotor.class,"leftRear"); //backleft, port 3
        rightRear = hardwareMap.get(DcMotor.class,"rightRear");  //backright, port 2
        rLift = hardwareMap.get(DcMotor.class,"rLift");
        lLift = hardwareMap.get(DcMotor.class,"lLift");
        leftIntake = hardwareMap.get(CRServo.class,"leftIntake");
        rightIntake = hardwareMap.get(CRServo.class,"rightIntake");
        intakein = hardwareMap.get(CRServo.class,"intakein");
        dread = hardwareMap.get(CRServo.class,"dread");
        //vector = hardwareMap.get(DcMotor.class,"vector");
        outtake = hardwareMap.get(Servo.class,"outtake");

        rightPull = hardwareMap.get(Servo.class, "rightPull");
        leftPull = hardwareMap.get(Servo.class, "leftPull");

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rLift.setDirection(DcMotor.Direction.FORWARD);
        lLift.setDirection(DcMotor.Direction.REVERSE);
        //vector.setDirection(DcMotor.Direction.FORWARD);
        //displayKind = Blink.DisplayKind.AUTO;

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(12.5, -68, Math.toRadians(270));



        Trajectory trajStart = drive.trajectoryBuilder(new Pose2d())
                .back(25.5)
                .build();

        TrajectorySequence traj1_1 = drive.trajectorySequenceBuilder(trajStart.end())
                .turn(Math.toRadians(90))
                .build();

        Trajectory traj1_2 = drive.trajectoryBuilder(traj1_1.end())
                .back(1)
                .build();

        Trajectory traj1_3 = drive.trajectoryBuilder(traj1_1.end())
                .forward(5)
                .build();

        TrajectorySequence traj1_4 = drive.trajectorySequenceBuilder(traj1_3.end())
                .turn(Math.toRadians(180))
                .build();

        Trajectory traj1_5 = drive.trajectoryBuilder(traj1_4.end())
                .strafeRight(22)
                .build();

        Trajectory traj1_6 = drive.trajectoryBuilder(traj1_5.end())
                .back(87)
                .build();


        Trajectory traj2_1 = drive.trajectoryBuilder(new Pose2d())
                .back(29)
                .build();
        Trajectory traj2_2 = drive.trajectoryBuilder(traj2_1.end())
                .forward(8)
                .build();
        TrajectorySequence traj2_3 = drive.trajectorySequenceBuilder(traj2_2.end())
                .turn(Math.toRadians(-90))
                .build();
        Trajectory traj2_4 = drive.trajectoryBuilder(traj2_3.end())
                .forward(15)
                .build();
        Trajectory traj2_5 = drive.trajectoryBuilder(traj2_4.end())
                .strafeRight(28)
                .build();
        Trajectory traj2_6 = drive.trajectoryBuilder(traj2_5.end())
                .back(98)
                .build();

        Trajectory trajStart3 = drive.trajectoryBuilder(new Pose2d())
                .back(30.5)
                .build();

        TrajectorySequence traj3_1 = drive.trajectorySequenceBuilder(trajStart3.end())
                .turn(Math.toRadians(270))
                .build();
        Trajectory traj3_2 = drive.trajectoryBuilder(traj3_1.end())
                .back(8)
                .build();
        Trajectory traj3_3 = drive.trajectoryBuilder(traj3_2.end())
                .forward(15)
                .build();
        Trajectory traj3_4 = drive.trajectoryBuilder(traj3_3.end())
                .strafeRight(17)
                .build();
        Trajectory traj3_5 = drive.trajectoryBuilder(traj3_4.end())
                .back(97)
                .build();


        if (opModeIsActive()) {
            while (opModeIsActive()) {



                telemetryTfod();
                // Push telemetry to the Driver Station.
                telemetry.update();

                if (spikeLocation() == 3) {


                    drive.followTrajectory(trajStart3);
                    drive.followTrajectorySequence(traj3_1);
                    drive.followTrajectory(traj3_2);
                    drive.followTrajectory(traj3_3);
                    drive.followTrajectory(traj3_4);
                    drive.followTrajectory(traj3_5);
                    /*
                    drive.followTrajectorySequence(traj_wait3);
                    drive.followTrajectory(traj3_6);
                    drive.followTrajectory(traj3_7);*/
                } else if (spikeLocation() == 2) {
                    drive.followTrajectory(traj2_1);
                    drive.followTrajectory(traj2_2);
                    drive.followTrajectorySequence(traj2_3);
                    drive.followTrajectory(traj2_4);
                    drive.followTrajectory(traj2_5);
                    drive.followTrajectory(traj2_6);
                } else {
                    drive.followTrajectory(trajStart);
                    drive.followTrajectorySequence(traj1_1);
                    //drive.followTrajectory(traj1_2);
                    drive.followTrajectory(traj1_3);
                    drive.followTrajectorySequence(traj1_4);
                    drive.followTrajectory(traj1_5);
                    drive.followTrajectory(traj1_6);
                    //CODE TO DEPOSIT PRELOAD ON LEFT SPIKE MARK
                    //ORIENT ROBOT
                }



                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);

                sleep(30000);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    private double spikeLocation() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        double location = 1;

        for (Recognition recognition : currentRecognitions) {

            if (recognition.getLeft() <= 386) {
                location = 2;
                telemetry.addData("Spike mark location: ", "center");
            } else if (recognition.getLeft() > 386) {
                location = 3;
                telemetry.addData("Spike mark location: ", "right");
            } else {
                location = 1;
                telemetry.addData("Spike mark location: ", "left");
            }

        }   // end for() loop

        return location;
    }

    public void driveForward(double distance, double power) {

        //Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);

        while (rightFront.getCurrentPosition() < distance) {
            telemetry.addData("Left Encoder", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(500);

    }

    public void driveBackward(double distance, double power) {

        //Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power * -1);
        rightFront.setPower(power * -1);
        leftRear.setPower(power * -1);
        rightRear.setPower(power * -1);

        while (-rightFront.getCurrentPosition() < distance) {
            telemetry.addData("Left Encoder", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(500);

    }

    public void strafeRight(double distance, double power) {

        //Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        rightFront.setPower(power * -1);
        leftRear.setPower(power * -1);
        rightRear.setPower(power);

        while (-rightFront.getCurrentPosition() < distance) {
            telemetry.addData("Left Encoder", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(500);

    }

    public void strafeLeft(double distance, double power) {

        //Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power * -1);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power * -1);

        while (rightFront.getCurrentPosition() < distance) {
            telemetry.addData("Left Encoder", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(500);

    }



    public void turnRight(double distance, double power) {

        //Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        rightFront.setPower(power * -1);
        leftRear.setPower(power);
        rightRear.setPower(power * -1);

        while (-rightFront.getCurrentPosition() < distance) {
            telemetry.addData("Left Encoder", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(500);

    }

    public void turnLeft(double distance, double power) {

        //Reset Encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power * -1);
        rightFront.setPower(power);
        leftRear.setPower(power * -1);
        rightRear.setPower(power);

        while (rightFront.getCurrentPosition() < distance) {
            telemetry.addData("Left Encoder", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        sleep(500);

    }

    public void dreadOut(int time) {

        dread.setPower(-1);
        sleep(time);
        dread.setPower(0);

        sleep(500);

    }

    public void backdropDeposit() {
        outtake.setPosition(0);
        sleep(1000);
        driveForward(50,0.3);
        sleep(500);
        outtake.setPosition(1);

    }

    public void dreadIn(int time) {

        dread.setPower(1);
        sleep(time);
        dread.setPower(0);

        sleep(500);

    }

    public void frontDeposit() {
        rightIntake.setPower(1);

        sleep(1000);

        rightIntake.setPower(0);
    }

}   // end class
