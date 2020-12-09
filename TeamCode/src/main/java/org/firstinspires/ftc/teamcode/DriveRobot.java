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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;
        double leftDiscLauncherPower = 0;
        double rightDiscLauncherPower = 0;
        double currentServoPos = 0 ;

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
            leftDiscLauncherPower = 0.5;
            rightDiscLauncherPower = 0.5;
        }

        currentServoPos = robot.horizontalTurret.getPosition();

        if(gamepad1.y && !buttonY && currentServoPos < 1) {
            robot.horizontalTurret.setPosition(currentServoPos + 0.05);
            buttonY = true;
        }
        else if (!gamepad1.y) {
            buttonY = false;
        }

        if(gamepad1.b && !buttonB && currentServoPos > 0) {
            robot.horizontalTurret.setPosition(currentServoPos - 0.05);
            buttonB = true;
        }
        else if (!gamepad1.b) {
            buttonB = false;
        }

        robot.leftFrontDrive.setPower(leftFrontPower * driveSpeed);
        robot.rightFrontDrive.setPower(rightFrontPower * driveSpeed);
        robot.leftBackDrive.setPower(leftBackPower * driveSpeed);
        robot.rightBackDrive.setPower(rightBackPower * driveSpeed);
        robot.leftDiscLauncher.setPower(leftDiscLauncherPower);
        robot.rightDiscLauncher.setPower(rightDiscLauncherPower);

        telemetry.addData("Left Front Power",leftFrontPower * driveSpeed);
        telemetry.addData("Right Front Power",rightFrontPower * driveSpeed);
        telemetry.addData("Left Back Power",leftBackPower * driveSpeed);
        telemetry.addData("Right Back Power",rightBackPower * driveSpeed);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Disc Launcher Speed", leftDiscLauncherPower);
        telemetry.addData("Right Launcher Speed", rightDiscLauncherPower);


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
