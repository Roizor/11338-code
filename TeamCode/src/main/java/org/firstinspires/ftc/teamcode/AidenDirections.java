package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public enum AidenDirections {
    FORWARDS,
    BACKWARDS,
    LEFT,
    RIGHT,
    OPEN,
    CLOSED;
    public static final DcMotorSimple.Direction MFORWARD = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction REVERSE = DcMotorSimple.Direction.REVERSE;
}