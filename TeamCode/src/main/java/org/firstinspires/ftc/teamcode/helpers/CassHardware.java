/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.AidenDirections;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Stack;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class CassHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor Bob   = null;
    public DcMotor Timothy  = null;
    public DcMotor Bleff   = null;
    public DcMotor Josh  = null;
    public DcMotor Evel = null;

        /*
    bob front left
    timothy front right
    bleff back left
    josh back right
     */

    public BNO055IMU imu = null;
    public AidenDirections dirs;
    public Servo LeftHand = null;
    private double inchesToBot(double inches) {
        return inches * 32.1934294;
    }
    //nice
    public Servo RightHand = null;
    public WebcamName Cammy = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    // public static final double MID_SERVO       =  0.5 ;
    //public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public CassHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public Stack<String> telemetryQueue;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        Bob  = myOpMode.hardwareMap.get(DcMotor.class, "lefron");
        Timothy = myOpMode.hardwareMap.get(DcMotor.class, "rifron");
        Bleff   = myOpMode.hardwareMap.get(DcMotor.class, "bleff");
        Josh = myOpMode.hardwareMap.get(DcMotor.class, "bright");
        Evel = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        LeftHand = myOpMode.hardwareMap.get(Servo.class, "leftc");
        RightHand = myOpMode.hardwareMap.get(Servo.class, "rightc");
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        Cammy = myOpMode.hardwareMap.get(WebcamName.class, "Cammy");

        resetEncoders();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        Bob.setDirection(DcMotor.Direction.REVERSE);
        Timothy.setDirection(DcMotor.Direction.FORWARD);
        Bleff.setDirection(DcMotor.Direction.REVERSE);
        Josh.setDirection(DcMotor.Direction.FORWARD);

        Bob.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Timothy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bleff.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Josh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = true;
        imu.initialize(imuParameters);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //  leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        //  rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        //   leftHand.setPosition(MID_SERVO);
        //   rightHand.setPosition(MID_SERVO);
    }

    /**
     * Definitely an experimental feature.
     * @param zpb ZeroPowerBehavior
     */
    public void setZPB(DcMotor.ZeroPowerBehavior zpb) {
        Bob.setZeroPowerBehavior(zpb);
        Timothy.setZeroPowerBehavior(zpb);
        Bleff.setZeroPowerBehavior(zpb);
        Josh.setZeroPowerBehavior(zpb);
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double Drive, double Turn, double strafe) {

        // Combine drive and turn for blended motion.
        double le = Drive + Turn + strafe;
        double ri = Drive - Turn - strafe;
        double lb = Drive + Turn - strafe;
        double rb = Drive - Turn + strafe;

        // Scale the values so neither exceed +/- 1.0

        // Use existing function to drive both wheels.
        setDrivePower(le, ri ,lb ,rb);
    }

    /**
     * Turn a direction
     * @param direction the direction you want to turn (left, right)
     * @param strafe fine tune how much power youwant
     * @implNote This is an experimental function!!
     */
    public void turn(AidenDirections direction, double strafe) {
        double Turn = 0;
        double presetDrive = 0.2;
        if(direction == AidenDirections.LEFT) Turn = -0.5;
        if(direction == AidenDirections.RIGHT) Turn = 0.5;
        double le = presetDrive + Turn + strafe;
        double ri = presetDrive + Turn - strafe;
        double lb = presetDrive + Turn - strafe;
        double rb = presetDrive + Turn + strafe;

        this.setDrivePower(le, ri ,lb ,rb);
    }
    public void addTelemetryData(String prompt, Object value) {
        telemetryQueue.push(prompt+":"+value);
    }

    public void resetEncoders() {
        Bob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Timothy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bleff.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Josh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Bob.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Timothy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bleff.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Josh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setAllDirections(DcMotor.Direction bobdir, DcMotor.Direction timdir, DcMotor.Direction bleffdir, DcMotor.Direction joshdir) {
        Bob.setDirection(bobdir);
        Timothy.setDirection(timdir);
        Bleff.setDirection(bleffdir);
        Josh.setDirection(joshdir);
    }

    public void setHand(AidenDirections position) {
        if(position == AidenDirections.OPEN) {
            LeftHand.setPosition(0.2);
            RightHand.setPosition(0.25);
        } else if(position == AidenDirections.ALLOPEN) {
            LeftHand.setPosition(0.44);
            RightHand.setPosition(0.07);
        } else if(position == AidenDirections.CLOSED) {
            LeftHand.setPosition(0.11);
            RightHand.setPosition(0.4);
        }
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     */
    public void setDrivePower(double BobWheel, double TimothyWheel ,double BleffWheel ,double JoshWheel ) {
        // Output the values to the motor drives.
        Bob.setPower(BobWheel/2);
        Timothy.setPower(TimothyWheel/2);
        Bleff.setPower(BleffWheel/2);
        Josh.setPower(JoshWheel/2);
    }

    /**
     * Drive a certain distance.
     *
     * @param inches    The amount of inches you want to move
     * @param direction The direction you want to move in (ADirections)
     */
    public void driveDistance(double inches, AidenDirections direction) {
        double targetposition = inchesToBot(inches);
        double power = 0.45;
        if (direction == AidenDirections.BACKWARDS) {
            setAllDirections(AidenDirections.MFORWARD, AidenDirections.REVERSE, AidenDirections.MFORWARD, AidenDirections.REVERSE);
        } else if (direction == AidenDirections.LEFT) {
            setAllDirections(AidenDirections.MFORWARD, AidenDirections.MFORWARD, AidenDirections.REVERSE, AidenDirections.REVERSE);
        } else if (direction == AidenDirections.RIGHT) {
            setAllDirections(AidenDirections.REVERSE, AidenDirections.REVERSE, AidenDirections.MFORWARD, AidenDirections.MFORWARD);
        }
        while (Bob.getCurrentPosition() < targetposition && Timothy.getCurrentPosition() < targetposition && Bleff.getCurrentPosition() < targetposition && Josh.getCurrentPosition() < targetposition) {
            Bob.setPower(power);
            Timothy.setPower(power);
            Bleff.setPower(power);
            Josh.setPower(power);
        }
        setAllDirections(AidenDirections.REVERSE, AidenDirections.MFORWARD, AidenDirections.REVERSE, AidenDirections.MFORWARD);
        Bob.setPower(0);
        Timothy.setPower(0);
        Bleff.setPower(0);
        Josh.setPower(0);
        resetEncoders();
    }
}
