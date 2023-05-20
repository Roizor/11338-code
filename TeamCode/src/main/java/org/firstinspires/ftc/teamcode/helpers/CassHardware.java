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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is the hardware map.
 * It does hardware mapping things for easier control over motors & servos.
 */

public class CassHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them public so they can be accessed externally)
    public DcMotor motorLeftFront = null;
    public DcMotor motorRightFront = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorLift = null;

        /*
    bob front left
    timothy front right
    bleff back left
    josh back right
     */

    public IMU imu = null;
    public AidenDirections dirs; // Custom direction enumerator for easier readability
    public Servo servoLeftHand = null;

    /**
     * Turn your set amount of inches into motor ticks today!
     * @param inches The amount of inches you want to... move!
     * @return Returns a fully converted inches to tick number that can be used for... moving the motors.
     */
    private double inchesToBot(double inches) {
        return inches * 32.1934294;
    }
    public Servo servoRightHand = null;
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
        motorLeftFront = myOpMode.hardwareMap.get(DcMotor.class, "lefron");
        motorRightFront = myOpMode.hardwareMap.get(DcMotor.class, "rifron");
        motorBackLeft = myOpMode.hardwareMap.get(DcMotor.class, "bleff");
        motorBackRight = myOpMode.hardwareMap.get(DcMotor.class, "bright");
        motorLift = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        servoLeftHand = myOpMode.hardwareMap.get(Servo.class, "leftc");
        servoRightHand = myOpMode.hardwareMap.get(Servo.class, "rightc");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        Cammy = myOpMode.hardwareMap.get(WebcamName.class, "Cammy");

        // This is where most of the magical setup happens.
        resetEncoders();

        this.setAllDirections(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);

        this.setZPB(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    /**
     * Definitely an experimental feature.
     * This was never actually used.
     * @param zpb ZeroPowerBehavior
     */
    public void setZPB(DcMotor.ZeroPowerBehavior zpb) {
        motorLeftFront.setZeroPowerBehavior(zpb);
        motorRightFront.setZeroPowerBehavior(zpb);
        motorBackLeft.setZeroPowerBehavior(zpb);
        motorBackRight.setZeroPowerBehavior(zpb);
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
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set directions of every motor in one function!
     * @param bobdir DcMotor direction you want Bob to move
     * @param timdir DcMotor direction you want Timothy to move
     * @param bleffdir DcMotor direction you want Bleff to move
     * @param joshdir DcMotor direction you want Josh to move
     */
    public void setAllDirections(DcMotor.Direction bobdir, DcMotor.Direction timdir, DcMotor.Direction bleffdir, DcMotor.Direction joshdir) {
        motorLeftFront.setDirection(bobdir);
        motorRightFront.setDirection(timdir);
        motorBackLeft.setDirection(bleffdir);
        motorBackRight.setDirection(joshdir);
    }

    /**
     * Set the hand position according to the AidenDirections Enumerator
     * @param position Any of the following: AidenDirections.[OPEN, ALLOPEN, CLOSED]
     */
    public void setHand(AidenDirections position) {
        if(position == AidenDirections.OPEN) {
            servoLeftHand.setPosition(0.2);
            servoRightHand.setPosition(0.25);
        } else if(position == AidenDirections.ALLOPEN) {
            servoLeftHand.setPosition(0.44);
            servoRightHand.setPosition(0.07);
        } else if(position == AidenDirections.CLOSED) {
            servoLeftHand.setPosition(0.09);
            servoRightHand.setPosition(0.4);
        }
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     */
    public void setDrivePower(double BobWheel, double TimothyWheel ,double BleffWheel ,double JoshWheel ) {
        // Output the values to the motor drives.
        motorLeftFront.setPower(BobWheel/2);
        motorRightFront.setPower(TimothyWheel/2);
        motorBackLeft.setPower(BleffWheel/2);
        motorBackRight.setPower(JoshWheel/2);
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
        while (motorLeftFront.getCurrentPosition() < targetposition && motorRightFront.getCurrentPosition() < targetposition && motorBackLeft.getCurrentPosition() < targetposition && motorBackRight.getCurrentPosition() < targetposition) {
            motorLeftFront.setPower(power);
            motorRightFront.setPower(power);
            motorBackLeft.setPower(power);
            motorBackRight.setPower(power);
        }
        setAllDirections(AidenDirections.REVERSE, AidenDirections.MFORWARD, AidenDirections.REVERSE, AidenDirections.MFORWARD);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        resetEncoders();
    }
}
