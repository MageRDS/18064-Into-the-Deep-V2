// BadBlueBackstage

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@Autonomous(name = "RedAudience", group = "Concept")
public class RedAudience extends LinearOpMode {

    private DcMotorEx leftFront = null;
    private DcMotorEx rightRear = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotor Intake = null;

    private CRServo wheel_bucket;

    private Servo left_servo_lift;

    private Servo right_servo_lift;

    private Servo flipper_bucket;

    private DcMotor slide = null;
    private CRServo drone = null;

    private DcMotor left_lift = null;

    private DcMotor right_lift = null;




    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "red_hatv3.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Red_hat.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "red_hat",
            "r",
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

        initTfod();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        slide = hardwareMap.get(DcMotor.class, "slide");
        left_lift = hardwareMap.get(DcMotor.class, "left_lift");
        right_lift =  hardwareMap.get(DcMotor.class, "right_lift");


        wheel_bucket = hardwareMap.get(CRServo.class, "wheel_bucket"); // Port 5 Expansion Hub
        flipper_bucket = hardwareMap.get(Servo.class, "flipper_bucket"); // port 4 Expansion Hub
        drone = hardwareMap.get(CRServo.class, "drone");

        left_servo_lift = hardwareMap.get(Servo.class, "left_servo_lift");
        right_servo_lift = hardwareMap.get(Servo.class, "right_servo_lift");



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.REVERSE);




        drone.setDirection(DcMotorSimple.Direction.FORWARD);


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        //---------------------ID 1 Trajectories-----------------



        Trajectory left_traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(30)
                .build();

        TrajectorySequence left_traj2 = drive.trajectorySequenceBuilder(left_traj1.end())
                .turn(Math.toRadians(-270))
                .build();

        Trajectory left_traj3 = drive.trajectoryBuilder(left_traj2.end())
                .back(4)
                .build();
        Trajectory left_traj4 = drive.trajectoryBuilder(left_traj3.end())
                .forward(4)
                .build();

        Trajectory left_traj5 = drive.trajectoryBuilder(left_traj4.end())
                .strafeLeft(26)
                .build();
        Trajectory left_traj6 = drive.trajectoryBuilder(left_traj5.end())
                .forward(80)
                .build();
        Trajectory left_traj7 = drive.trajectoryBuilder(left_traj6.end())
                .strafeRight(5)
                .build();
        // --------------------



        // -------------------Middle Trajectories-------------
        Trajectory middle_traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(30)
                .build();
        Trajectory middle_traj2 = drive.trajectoryBuilder(middle_traj1.end())
                .forward(5)
                .build();
        Trajectory middle_traj3 = drive.trajectoryBuilder(middle_traj2.end())
                .strafeRight(15)
                .build();
        Trajectory middle_traj4 = drive.trajectoryBuilder(middle_traj3.end())
                .back(25)
                .build();
        TrajectorySequence middle_traj5 = drive.trajectorySequenceBuilder(middle_traj4.end())
                .turn(Math.toRadians(-91))
                .build();
        Trajectory middle_traj6 = drive.trajectoryBuilder(middle_traj5.end())
                .back(100)
                .build();
        Trajectory middle_traj7 = drive.trajectoryBuilder(middle_traj5.end())
                .strafeLeft(5)
                .build();
        // -------------------- ID 3 Trajectories -----------
        Trajectory right_traj1 = drive.trajectoryBuilder(new Pose2d())
                .back(25)
                .build();
        TrajectorySequence right_traj2 = drive.trajectorySequenceBuilder(right_traj1.end())
                .turn(Math.toRadians(-90))
                .build();
        Trajectory right_traj3 = drive.trajectoryBuilder(right_traj2.end())
                .back(5)
                .build();
        Trajectory right_traj4 = drive.trajectoryBuilder(right_traj3.end())
                .forward(10)
                .build();
        Trajectory right_traj5 = drive.trajectoryBuilder(right_traj4.end())
                .strafeRight(23)
                .build();

        Trajectory right_traj7 = drive.trajectoryBuilder(right_traj5.end())
                .back(100)
                .build();
        Trajectory right_traj8 = drive.trajectoryBuilder(right_traj7.end())
                .strafeLeft(5)
                .build();





        // Camera Stuff Don't edit

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();



                sleep(7000);
                if (spikeLocation() == 3) {

                    drive.followTrajectory(right_traj1);
                    drive.followTrajectorySequence(right_traj2);
                    drive.followTrajectory(right_traj3);
                    drive.followTrajectory(right_traj4);
                    sleep(800);
                    drive.followTrajectory(right_traj5);
                    sleep(200);
                    drive.followTrajectory(right_traj7);
                    drive.followTrajectory(right_traj8);

                    sleep(100000);


                } else if (spikeLocation() == 2) {

                    drive.followTrajectory(middle_traj1);
                    drive.followTrajectory(middle_traj2);
                    sleep(800);
                    drive.followTrajectory(middle_traj3);
                    drive.followTrajectory(middle_traj4);
                    drive.followTrajectorySequence(middle_traj5);
                    drive.followTrajectory(middle_traj6);
                    drive.followTrajectory(middle_traj7);


                    sleep(100000);



                } else {

                    drive.followTrajectory(left_traj1);
                    sleep(200);
                    drive.followTrajectorySequence(left_traj2);
                    sleep(200);
                    drive.followTrajectory(left_traj3);
                    sleep(200);
                    drive.followTrajectory(left_traj4);
                    sleep(800);
                    drive.followTrajectory(left_traj5);
                    sleep(400);
                    drive.followTrajectory(left_traj6);
                    sleep(400);
                    drive.followTrajectory(left_traj7);





                    sleep(100000);

                }



                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
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
        tfod.setMinResultConfidence(0.80f);

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

    public void intake(String mode, double power){
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (mode == "intake"){
            Intake.setPower(power);
        }

        if (mode == "outtake"){
            Intake.setPower(-power);
        }
        if (mode == "stop"){
            Intake.setPower(0);
        }

    }
    private double spikeLocation() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        double location = 1;

        for (Recognition recognition : currentRecognitions) {

            if (recognition.getLeft() <= 322) {
                location = 2;
                telemetry.addData("Spike mark location: ", "center");
            } else if (recognition.getLeft() > 322) {
                location = 3;
                telemetry.addData("Spike mark location: ", "right");
            } else {
                location = 1;
                telemetry.addData("Spike mark location: ", "left");
            }

        }   // end for() loop

        return location;
    }


    public void armDown(double distance, double power) {

        //Reset Encoders
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        slide.setPower(-power);


        while (-slide.getCurrentPosition() < distance) {
            telemetry.addData("Arm Encoder", slide.getCurrentPosition());
            telemetry.update();
        }

        slide.setPower(0);

        sleep(500);

    }


    public void armUp(double power, String mode) {

        //Reset Encoders
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setPower(power);
        if (mode == "Up"){
            slide.setPower(power);
        }

        if (mode == "Down"){
            slide.setPower(-power);
        }
        if (mode == "stop"){
            slide.setPower(0);
        }



        slide.setPower(0);

        sleep(1000);

    }

} // end class
