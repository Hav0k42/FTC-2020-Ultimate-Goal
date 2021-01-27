/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Drive Robot", group="Opmode")
public class DriveRobot extends OpMode {




    //Camera variables
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    OpenGLMatrix robotFromCamera;


    //declare trackables here, because of scope issues.
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
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





    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    HardwareConfig robot = new HardwareConfig();
    double initialDiscVelocity = 20.0; //speed of the disc cannon in meters per second.
    double cannonRadius = 0.2; //radius of the cannon in meters.
    double driveSpeed = 1.0;
    int driveSpeedFlag = 0;

    boolean buttonY = false;
    boolean buttonB = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        webcamName = hardwareMap.get(WebcamName.class, "cam");


        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
//        redAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        blueAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//        frontWallTarget.setLocation(OpenGLMatrix
//                .translation(-halfField, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
//        blueTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//        redTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

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

        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsUltimateGoal.activate();



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double yAxisValue = 0;
    double zAxisValue = 0;
    float centeredValue;
    int horizontalServoSearchDirection = 0;//0: left, 1: right
    double lastOrientation = 0;

    double currentServoPos = 0 ;


    //the following variable names will require references to the diagram drawn with the associated calculations.
    float triangleSideA = 35.5f;
    float triangleSideB = 70.5f;
    float triangleSideC = (float)Math.sqrt(Math.pow(triangleSideA, 2) + Math.pow(triangleSideB, 2));

    float triangleAngleZ = (float)Math.asin(triangleSideA / triangleSideC);

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry


        String activeTarget = "";
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
            yAxisValue = translation.get(1) / mmPerInch;
            zAxisValue = translation.get(0) / mmPerInch;
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, INTRINSIC, XYZ, DEGREES);
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












        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;
        double DiscLauncherPower = 0;
        double rightDiscLauncherPower = 0;

        leftFrontPower = -Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.75), -1.0, 1.0);
        rightFrontPower = -Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.75), -1.0, 1.0);
        leftBackPower = -Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.75), -1.0, 1.0);
        rightBackPower = -Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * 0.75), -1.0, 1.0);


        if(gamepad1.right_trigger > 0) {
            driveSpeed = 1 - gamepad1.right_trigger;
            if(driveSpeed < 0.3) {
                driveSpeed = 0.3;
            }
        }
            

        if(gamepad1.a) {
            DiscLauncherPower = 0.5;
        }

        if (gamepad1.b && activeTarget.equals("Red Tower Goal Target")) {
            double angleToFireFrom = calculateAngle(zAxisValue, 10);
            runtime.reset();
            while (runtime.seconds() < 2) {

            }
            robot.DiscLauncher.setPower(0.5);
            runtime.reset();
            while (runtime.seconds() < 0.2) {

            }
            robot.launcherServo.setPosition(1);
            runtime.reset();
            while (runtime.seconds() < 1) {

            }
            robot.launcherServo.setPosition(0.7);
            robot.DiscLauncher.setPower(0);
        }

//        if(gamepad1.y && !buttonY && currentServoPos < 1) {
//            robot.horizontalTurret.setPosition(currentServoPos + 0.05);
//            buttonY = true;
//        }
//        else if (!gamepad1.y) {
//            buttonY = false;
//        }
//
//        if(gamepad1.b && !buttonB && currentServoPos > 0) {
//            robot.horizontalTurret.setPosition(currentServoPos - 0.05);
//            buttonB = true;
//        }
//        else if (!gamepad1.b) {
//            buttonB = false;
//        }
        if(gamepad1.left_trigger && !buttonY && currentServoPos <1) {
            robot.launcherServo.setPosition(currentServoPos +.05);
            buttonY = true;
            
        else if (!gamepad1.left_trigger)
            buttonY = false;


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




        robot.leftFrontDrive.setPower(leftFrontPower * driveSpeed);
        robot.rightFrontDrive.setPower(rightFrontPower * driveSpeed);
        robot.leftBackDrive.setPower(leftBackPower * driveSpeed);
        robot.rightBackDrive.setPower(rightBackPower * driveSpeed);
        robot.DiscLauncher.setPower(DiscLauncherPower);

        robot.horizontalTurret.setPosition(currentServoPos);

        telemetry.addData("test", centeredValue);
        telemetry.addData("\nMotors:\nLeft Front Power",leftFrontPower * driveSpeed);
        telemetry.addData("Right Front Power",rightFrontPower * driveSpeed);
        telemetry.addData("Left Back Power",leftBackPower * driveSpeed);
        telemetry.addData("Right Back Power",rightBackPower * driveSpeed);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Disc Launcher Speed", DiscLauncherPower);
        telemetry.addData("\nServos:\nHorizontalServoPos", currentServoPos);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

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

}
