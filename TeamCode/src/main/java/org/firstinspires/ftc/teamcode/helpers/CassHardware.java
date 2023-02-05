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
 * This is the hardware map.
 * It does hardware mapping things for easier control over motors & servos.
 */

public class CassHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them public so they can be accessed externally)
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
    public AidenDirections dirs; // Custom direction enumerator for easier readability
    public Servo LeftHand = null;

    /**
     * Turn your set amount of inches into motor ticks today!
     * @param inches The amount of inches you want to... move!
     * @return Returns a fully converted inches to tick number that can be used for... moving the motors.
     */
    private double inchesToBot(double inches) {
        return inches * 32.1934294;
    }
    public Servo RightHand = null;
    public WebcamName Cammy = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public CassHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

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

        // This is where most of the magical setup happens.
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
    }

    /**
     * Definitely an experimental feature.
     * This was never actually used.
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

    /**
     * Reset the encoders on each motor (Set the runmode to stop & reset, then restart with encoders)
     */
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

    /**
     * Set directions of every motor in one function!
     * @param bobdir DcMotor direction you want Bob to move
     * @param timdir DcMotor direction you want Timothy to move
     * @param bleffdir DcMotor direction you want Bleff to move
     * @param joshdir DcMotor direction you want Josh to move
     */
    public void setAllDirections(DcMotor.Direction bobdir, DcMotor.Direction timdir, DcMotor.Direction bleffdir, DcMotor.Direction joshdir) {
        Bob.setDirection(bobdir);
        Timothy.setDirection(timdir);
        Bleff.setDirection(bleffdir);
        Josh.setDirection(joshdir);
    }

    /**
     * Set the hand position according to the AidenDirections Enumerator
     * @param position Any of the following: AidenDirections.[OPEN, ALLOPEN, CLOSED]
     */
    public void setHand(AidenDirections position) {
        if(position == AidenDirections.OPEN) {
            LeftHand.setPosition(0.2);
            RightHand.setPosition(0.25);
        } else if(position == AidenDirections.ALLOPEN) {
            LeftHand.setPosition(0.44);
            RightHand.setPosition(0.07);
        } else if(position == AidenDirections.CLOSED) {
            LeftHand.setPosition(0.09);
            RightHand.setPosition(0.4);
        }
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
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
