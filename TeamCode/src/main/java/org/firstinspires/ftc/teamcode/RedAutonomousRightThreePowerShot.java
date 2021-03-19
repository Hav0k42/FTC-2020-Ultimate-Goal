/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.lang.ref.PhantomReference;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
@Autonomous(name="Red Right Three Power Shot", group ="Autonomous")
public class RedAutonomousRightThreePowerShot extends LinearOpMode
{

    HardwareConfig         robot   = new HardwareConfig();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double lowerRingThreshold = 130;
    static final double higherRingThreshold = 142;

    OpenCvCamera webCam;
    SkystoneDeterminationPipeline pipeline;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY = "AUFWnZP/////AAABmcligD0AJEN9mfG9JJMTRMYeS9+96qtOUucRsn64Whzh6Mwwz1MWkgA3ZynZSIGk/LqIk+z7p/kQzdqD5xFQ9+T151HXofFN8oWZUljGsnQty80eK1n46McK4YRwUvQ1bNbRGPHWfv+Ytzn6+NTQe8oueKXWSbbYTjRrUxLmCcqdbtKMoXlf4+MVfXKMQHfBVOn1i/YVQIQXl5WMCewjSR3/SVe1b7aaDSsfV7YRxF/qZPwiUDT2YKcCbas8H/gJTAAsMR7KktcAfSVumIWC1fa1MQEZiIKc/VUYefKvmgO66DCCV+cUpI68uV7C735VS0szO5uyPno0H6s+x8zApvj8FVIh2Pm6Q2hcmz4/chZv";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override
    public void runOpMode()
    {

        String pos = "";
        int analysis = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });






        webcamName = hardwareMap.get(WebcamName.class, "cam");

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.







        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftBackDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightBackDrive.getCurrentPosition()
        );
        telemetry.update();


        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();
        double turretServoPosition = 0;
        int autonomousStep = 0;
        int preAutoStep = 0;
        while (opModeIsActive() && preAutoStep != 2) {
            robot.wobbleLockServo.setPosition(0);
            if (preAutoStep == 0) {
                turretServoPosition = 0.46;
                preAutoStep = 1;
                runtime.reset();
            }
            if (preAutoStep == 1) {

                pos = pipeline.position.toString();
                analysis = pipeline.getAnalysis();
                if (analysis > 110) {
                    if (runtime.seconds() > 1.5) {
                        preAutoStep = 2;
                    }

                }
            }
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position.toString());
            telemetry.update();
            robot.horizontalTurret.setPosition(turretServoPosition);
        }





        webCam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener(){
            @Override
            public void onClose() {

            }
        });

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsUltimateGoal.activate();

        double yAxisValue = 0;
        double zAxisValue = 0;
        float centeredValue = 0;
        int horizontalServoSearchDirection = 0;//0: left, 1: right
        double lastOrientation = 0;

        double currentServoPos = 0 ;
        //the following variable names will require references to the diagram drawn with the associated calculations.
        float triangleSideA = 35.5f;
        float triangleSideB = 70.5f;
        float triangleSideC = (float)Math.sqrt(Math.pow(triangleSideA, 2) + Math.pow(triangleSideB, 2));

        float triangleAngleZ = (float)Math.asin(triangleSideA / triangleSideC);
        double launcherServoPosition = 0.4;

        while (opModeIsActive())
        {
            String activeTarget = "";
            telemetry.addData("Analysis", analysis);
            telemetry.addData("Position", pos);



            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    activeTarget = trackable.getName();

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);//Z-axis: vertical, Y-axis: Perpendicular Distance, X-axis: Parallel Distance

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                centeredValue = rotation.secondAngle;
                centeredValue += 180f;
                if (centeredValue > 180) {
                    centeredValue -= 360;
                }
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();



            if (autonomousStep == 0) {//Drive to the middle of the goals
                robot.wobbleArm.setPower(-0.2);
                encoderDrive(0.75, -18, -18, -18, -18, 10);

                autonomousStep = 1;
            }//Drive to the middle of the goals

            if (analysis > higherRingThreshold) {//Furthest Square *Target C
                pos = "FOUR";
                telemetry.addData("Target", "C");
                if (autonomousStep == 1) {
                    encoderDrive(0.5, -3.3, -3.3, 3.3, 3.3, 10);







                    //scuttle to the side a bit
                    encoderDrive(0.5, -1.5, 1.5, 1.5, -1.5, 10);

                    runtime.reset();
                    robot.wobbleArm.setPower(0.8);
                    while (runtime.seconds() < 0.7) {}
                    robot.wobbleArm.setPower(0);
                    robot.wobbleLockServo.setPosition(1);
                    runtime.reset();
                    while (runtime.seconds() < 0.2) {}

                    //scuttle back
                    encoderDrive(0.5, 1.5, -1.5, -1.5, 1.5, 10);
                    robot.wobbleLockServo.setPosition(0);
                    runtime.reset();
                    while (runtime.seconds() < 0.1) {}
                    robot.wobbleArm.setPower(-0.8);
                    runtime.reset();
                    while (runtime.seconds() < 0.8) {}

                    robot.wobbleArm.setPower(0);








                    encoderDrive(0.5, 3.3, 3.3, -3.3, -3.3, 10);

                    autonomousStep = 2;
                }
            } else if (analysis < higherRingThreshold && analysis > lowerRingThreshold) {//Middle Square *Target B
                pos = "ONE";
                telemetry.addData("Target", "B");
                if (autonomousStep == 1) {
                    encoderDrive(0.5, 7.7, 7.7, -7.7, -7.7, 10);





                    //scuttle to the side a bit
                    encoderDrive(0.5, -1.5, 1.5, 1.5, -1.5, 10);

                    runtime.reset();
                    robot.wobbleArm.setPower(0.8);
                    while (runtime.seconds() < 0.7) {}
                    robot.wobbleArm.setPower(0);
                    robot.wobbleLockServo.setPosition(1);
                    runtime.reset();
                    while (runtime.seconds() < 0.2) {}

                    //scuttle back
                    encoderDrive(0.5, 1.5, -1.5, -1.5, 1.5, 10);
                    robot.wobbleLockServo.setPosition(0);
                    runtime.reset();
                    while (runtime.seconds() < 0.1) {}
                    robot.wobbleArm.setPower(-0.8);
                    runtime.reset();
                    while (runtime.seconds() < 0.8) {}

                    robot.wobbleArm.setPower(0);







                    encoderDrive(0.5, -7, -7, 7, 7, 10);
                    autonomousStep = 2;
                }
            } else {//Closest Square *Target A
                pos = "NONE";
                telemetry.addData("Target", "A");
                if (autonomousStep == 1) {
                    encoderDrive(0.5, 3.3, 3.3, -3.3, -3.3, 10);







                    //scuttle to the side a bit
                    encoderDrive(0.5, -2.5, 2.5, 2.5, -2.5, 10);

                    runtime.reset();
                    robot.wobbleArm.setPower(0.8);
                    while (runtime.seconds() < 0.7) {}
                    robot.wobbleArm.setPower(0);
                    robot.wobbleLockServo.setPosition(1);
                    runtime.reset();
                    while (runtime.seconds() < 0.2) {}

                    //scuttle back
                    encoderDrive(0.5, 2.5, -2.5, -2.5, 2.5, 10);
                    robot.wobbleLockServo.setPosition(0);
                    runtime.reset();
                    while (runtime.seconds() < 0.1) {}
                    robot.wobbleArm.setPower(-0.8);
                    runtime.reset();
                    while (runtime.seconds() < 0.8) {}

                    robot.wobbleArm.setPower(0);







                    encoderDrive(0.5, -2.7, -2.7, 2.7, 2.7, 10);
                    autonomousStep = 2;
                }
            }

            if (autonomousStep == 2 ) {//Drive behind the line, and shoot off 3 rings.

                if (analysis < lowerRingThreshold) {
                    encoderDrive(0.5, 3, -3, -3, 3, 10);
                    encoderDrive(0.75, 7.5, 7.5, 7.5, 7.5, 10);
                } else {
                    encoderDrive(0.75, 7.5, 7.5, 7.5, 7.5, 10);
                    encoderDrive(0.5, 3, -3, -3, 3, 10);
                }

                double angleToFireFrom = calculateAngle(zAxisValue, 10);
                robot.DiscLauncher.setPower(0.5);

                runtime.reset();
                while (runtime.seconds() < 3) {
                    if (targetVisible && activeTarget.equals("Red Tower Goal Target")) {//Robot sees the target under the red tower goal.
                        float targetCloseThreshold = 4.0f; //If the robot is aimed within this value, it is acceptable and will stop changing where it aims. This is so it doesn't swivel and look weird
//            if (Math.abs(centeredValue) < targetCloseThreshold) {
//                //robot is aimed at the right spot, or at least close enough. Do nothing.
//            } else
                        if (centeredValue > targetCloseThreshold) {
                            //robot turret needs to turn right.
                            horizontalServoSearchDirection = 0;
                            currentServoPos += (centeredValue * 0.00002); //The value the turret rotates by is proportional to the distance the picture is from being centered.
                        } else if (centeredValue < 0 - targetCloseThreshold) {
                            //robot turret needs to turn left.
                            horizontalServoSearchDirection = 1;
                            currentServoPos += (centeredValue * 0.00002); //The value the turret rotates by is proportional to the distance the picture is from being centered.
                        }

                        lastOrientation = robot.imu.getAngularOrientation().thirdAngle;//
                        telemetry.addData("Angle Rotation", lastOrientation);

                    } else if (targetVisible && activeTarget.equals("Red Alliance Target")) {
                        float triangleAngleX = centeredValue;//this angle is going to need to gotten from the vuforia stuff. Testing required to find out if its one of the ones provided, or if I need to do calculations for it.
                        float triangleSideD = (float)zAxisValue;//same with this side, except I know I don't need to do extra calculations.


                        float triangleAngleW = (float)(Math.PI - (triangleAngleZ + triangleAngleX));
                        float triangleSideE = (float)(Math.sqrt(Math.pow(triangleSideC, 2) + Math.pow(triangleSideD, 2)  - (2 * triangleSideC * triangleSideD * Math.cos(triangleAngleW))));
                        float triangleAngleV = (float)(Math.acos(-((Math.pow(triangleSideC, 2) - Math.pow(triangleSideE, 2) - Math.pow(triangleSideD, 2)) - (2 * triangleSideE * triangleSideD))));

                        float angleToTurn = (float)(triangleAngleV / Math.PI);

                        if (!Double.isNaN(angleToTurn)) {//make sure the value isn't imaginary.
                            currentServoPos -= angleToTurn;
                        }
                        if (currentServoPos > 1) {
                            currentServoPos = 1;
                        }
                        if (currentServoPos < 0) {
                            currentServoPos = 0;
                        }
                        horizontalServoSearchDirection = 1;
                    } else if (!targetVisible) {//Robot cannot see any targets.
                        if (horizontalServoSearchDirection == 0) {
                            currentServoPos += 0.0001;
                        }
                        if (horizontalServoSearchDirection == 1) {
                            currentServoPos -= 0.0001;
                        }
                        if (currentServoPos <= 0) {
                            currentServoPos = 0;
                            horizontalServoSearchDirection = 0;
                        }
                        if (currentServoPos >= 1) {
                            currentServoPos = 1;
                            horizontalServoSearchDirection = 1;
                        }
                    }//scan surroundings until correct target is found
                }

                if (!targetVisible) {
                    currentServoPos = 0.68;

                }
                telemetry.addData("targetFound", targetVisible);
                telemetry.update();
                telemetry.addData("targetFound", targetVisible);
                runtime.reset();
                while (runtime.seconds() < 3) {
                    robot.horizontalTurret.setPosition(currentServoPos - 0.04);
                }


                robot.launcherServo.setPosition(0.2);
                runtime.reset();
                while (runtime.seconds() < 0.5) {
                    robot.horizontalTurret.setPosition(currentServoPos - 0.14);
                }
                robot.launcherServo.setPosition(0);
                runtime.reset();
                while (runtime.seconds() < 1) {
                    robot.horizontalTurret.setPosition(currentServoPos - 0.14);
                    robot.collectionMotor.setPower(1);
                }
                robot.collectionMotor.setPower(0);


                robot.launcherServo.setPosition(0.2);
                runtime.reset();
                while (runtime.seconds() < 0.5) {
                    robot.horizontalTurret.setPosition(currentServoPos - 0.14);
                }
                robot.launcherServo.setPosition(0);
                runtime.reset();
                while (runtime.seconds() < 1) {
                    robot.horizontalTurret.setPosition(currentServoPos - 0.14);
                    robot.collectionMotor.setPower(1);
                }
                robot.collectionMotor.setPower(0);

                robot.DiscLauncher.setPower(0.45);


                robot.launcherServo.setPosition(0.2);
                runtime.reset();
                while (runtime.seconds() < 0.5) {
                    robot.horizontalTurret.setPosition(currentServoPos - 0.24);
                }
                robot.launcherServo.setPosition(0);
                runtime.reset();
                while (runtime.seconds() < 1) {
                    robot.horizontalTurret.setPosition(currentServoPos - 0.24);
                }
                robot.collectionMotor.setPower(0);




                robot.DiscLauncher.setPower(0);

                autonomousStep = 3;
            }

            if (autonomousStep == 3) {//drive forward until the color sensor sees white.
                encoderDrive(0.7, -3, -3, -3, -3, 2);

                autonomousStep = 4;

            }//drive forward until the color sensor sees white.


            if (targetVisible && activeTarget.equals("Red Tower Goal Target")) {//Robot sees the target under the red tower goal.
                float targetCloseThreshold = 4.0f; //If the robot is aimed within this value, it is acceptable and will stop changing where it aims. This is so it doesn't swivel and look weird
//            if (Math.abs(centeredValue) < targetCloseThreshold) {
//                //robot is aimed at the right spot, or at least close enough. Do nothing.
//            } else
                if (centeredValue > targetCloseThreshold) {
                    //robot turret needs to turn right.
                    horizontalServoSearchDirection = 0;
                    currentServoPos += (centeredValue * 0.00002); //The value the turret rotates by is proportional to the distance the picture is from being centered.
                } else if (centeredValue < 0 - targetCloseThreshold) {
                    //robot turret needs to turn left.
                    horizontalServoSearchDirection = 1;
                    currentServoPos += (centeredValue * 0.00002); //The value the turret rotates by is proportional to the distance the picture is from being centered.
                }

                lastOrientation = robot.imu.getAngularOrientation().thirdAngle;//
                telemetry.addData("Angle Rotation", lastOrientation);

            } else if (targetVisible && activeTarget.equals("Red Alliance Target")) {
                float triangleAngleX = centeredValue;//this angle is going to need to gotten from the vuforia stuff. Testing required to find out if its one of the ones provided, or if I need to do calculations for it.
                float triangleSideD = (float)zAxisValue;//same with this side, except I know I don't need to do extra calculations.


                float triangleAngleW = (float)(Math.PI - (triangleAngleZ + triangleAngleX));
                float triangleSideE = (float)(Math.sqrt(Math.pow(triangleSideC, 2) + Math.pow(triangleSideD, 2)  - (2 * triangleSideC * triangleSideD * Math.cos(triangleAngleW))));
                float triangleAngleV = (float)(Math.acos(-((Math.pow(triangleSideC, 2) - Math.pow(triangleSideE, 2) - Math.pow(triangleSideD, 2)) - (2 * triangleSideE * triangleSideD))));

                float angleToTurn = (float)(triangleAngleV / Math.PI);

                if (!Double.isNaN(angleToTurn)) {//make sure the value isn't imaginary.
                    currentServoPos -= angleToTurn;
                }
                if (currentServoPos > 1) {
                    currentServoPos = 1;
                }
                if (currentServoPos < 0) {
                    currentServoPos = 0;
                }
                horizontalServoSearchDirection = 1;
            } else if (!targetVisible) {//Robot cannot see any targets.
                if (horizontalServoSearchDirection == 0) {
                    currentServoPos += 0.0001;
                }
                if (horizontalServoSearchDirection == 1) {
                    currentServoPos -= 0.0001;
                }
                if (currentServoPos <= 0) {
                    currentServoPos = 0;
                    horizontalServoSearchDirection = 0;
                }
                if (currentServoPos >= 1) {
                    currentServoPos = 1;
                    horizontalServoSearchDirection = 1;
                }
            }//scan surroundings until correct target is found


            robot.horizontalTurret.setPosition(currentServoPos);

            /*
             * Autonomous Steps:
             * 1: Scan the rings to see which position the wobble goal should go
             * 2: Deliver the wobble goal to the correct zone
             *   a: no rings, deliver goal to target a
             *       Drive forward to target a
             *   b: one ring, deliver goal to target b
             *       Drive forward to halfway between targets a and c, turn in place, and drive forwards to deliver the goal to the right.
             *       Reset position by driving back the same amount, and turning in place backwards the same amount.
             *   c: four rings, deliver goal to target c
             *       Drive forward to target c
             *
             * 3: Drive back to the launch zone, and launch the rings into the top goal using the algorithm I've written to determine the correct angle.
             * 4: Drive forward to park on the line.
             *
             * */

        }





    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontDrive.setPower(Math.abs(speed));
            robot.leftBackDrive.setPower(Math.abs(speed));
            robot.rightFrontDrive.setPower(Math.abs(speed));
            robot.rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newLeftBackTarget,  newRightFrontTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.leftBackDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition(),
                        robot.rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }



    public static double calculateAngle(double horizontalDistance, double initVelocity) {//Calculate the angle the cannon should be using sensors. Make sure calculations are in m/s
        double verticalDistance = 0.90963725;
        double pi = 3.141592653589793238462643383;

        double a = 1;
        double b = -((initVelocity * initVelocity) / (4.905 * horizontalDistance));
        double c = -(((verticalDistance / horizontalDistance) + ((4.905 * horizontalDistance) / (initVelocity * initVelocity))) / ((-4.905 * horizontalDistance) / (initVelocity * initVelocity)));



        double anglePlus = Math.atan((-b + Math.sqrt((b * b) - (4 * a * c))) / (2 * a));
        double angleMinus = Math.atan((-b - Math.sqrt((b * b) - (4 * a * c))) / (2 * a));

        //Math.atan uses radians. Angle will be returned in radians.

        boolean plusWorks = false;
        boolean minusWorks = false;

        if (anglePlus > 0 && anglePlus < (pi / 2)) {
            plusWorks = true;
        }

        if (angleMinus > 0 && angleMinus < (pi / 2)) {
            minusWorks = true;
        }

        if (plusWorks && minusWorks) {
            return (Math.min(anglePlus, angleMinus));
        } else if (plusWorks) {
            return anglePlus;
        } else {
            return angleMinus;
        }

    }

    public static double distillAngle(double horizontalDistance, double initVelocity, double radius, double previousAngle, int passes) {
        while (passes != 0) {
            double verticalDistance = 0.90963725;
            double pi = 3.141592653589793238462643383;

            double a = ((-4.905 * (horizontalDistance - (radius * Math.cos(previousAngle))) * (horizontalDistance - (radius * Math.cos(previousAngle)))) / (initVelocity * initVelocity));
            double b = (horizontalDistance - (radius * Math.cos(previousAngle)));
            double c = -(verticalDistance - (radius * Math.sin(previousAngle)) + ((4.905 * (horizontalDistance - (radius * Math.cos(previousAngle))) * (horizontalDistance - (radius * Math.cos(previousAngle)))) / (initVelocity * initVelocity)));

            double anglePlus = Math.atan((-b + Math.sqrt((b * b) - (4 * a * c))) / (2 * a));
            double angleMinus = Math.atan((-b - Math.sqrt((b * b) - (4 * a * c))) / (2 * a));



            boolean plusWorks = false;
            boolean minusWorks = false;

            if (anglePlus > 0 && anglePlus < (pi / 2)) {
                plusWorks = true;
            }

            if (angleMinus > 0 && angleMinus < (pi / 2)) {
                minusWorks = true;
            }
            if (previousAngle == anglePlus || previousAngle == angleMinus) {
                passes = 1;
            } else {//Angle isn't yet refined enough
                if (plusWorks && minusWorks) {
                    previousAngle = (Math.min(anglePlus, angleMinus));
                } else if (plusWorks) {
                    previousAngle = anglePlus;
                } else {
                    previousAngle = angleMinus;
                }
            }
            passes--;
        }
        return previousAngle;
    }

    public static double angleCalc(double horizontalDistance, double initVelocity, double radius, int passes) {
        double firstAngle = calculateAngle(horizontalDistance, initVelocity);
        return distillAngle(horizontalDistance, initVelocity, radius, firstAngle, passes);
    }








    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);


        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(160,98);

        static final int REGION_WIDTH = 60;
        static final int REGION_HEIGHT = 40;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    GREEN, // The color the rectangle is drawn in
//                    -1); // Negative thickness means solid fill



            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

    }
}
