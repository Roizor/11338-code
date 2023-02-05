package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
// This is a custom enumerator created because I wanted a simpler way to read & write code.
// Each value corresponds to it's own value in a function so this file is only used to declare the enumerator.

/**
 * Direction enumerator.
 */
public enum AidenDirections {
    FORWARDS,
    BACKWARDS,
    ALLOPEN,
    LEFT,
    RIGHT,
    OPEN,
    CLOSED;
    public static final DcMotorSimple.Direction MFORWARD = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction REVERSE = DcMotorSimple.Direction.REVERSE;
}