include("robot.jl")

struct MotionArc
    radiusInches::Float32
    angularVelocityDegpS::Float32
end

function simulateStraightMovement(robot::Robot, targetVelocityInpS::Float32)
    
end

function simulateTurningCircle(robot::Robot, targetMotion::MotionArc)
    # Note that the outer wheel will rotate faster if we are going CCW (true in this test case),
    # and so encoders or resolver readings would show that difference in velocity. We are responsible
    # for simulating sensor inputs that would account for that difference.
    #
    # Example derivation of individual wheel velocities:
    # let the desired angular velocity be +200 deg/s (3.4906 rad/s), with a turning circle radius of 60 inches about the
    # center of the robot body.
    # The robot's track width is 24 inches, and its wheel radius is 6 inches.
    #
    # Because CCW angles are noted as positive in this reference frame, the left wheel of the robot
    # is the "inner wheel" during turns and therefore has a slightly smaller turning circle radius (60 - 24/2) = 48 inches
    # (compared to the right wheel's (60 + 24/2) = 72 inches).
    # Calculating the tangential velocity of each wheel will inform us "how much ground the wheel must cover" to
    # meet the target 200 deg/s angular velocity; for the left wheel, the tangential velocity would be
    # r * \omega = 3.4906 rad/s * 48 in = 167.5516 in/s. To convert this to the wheel's angular velocity, we then
    # have to identify how many revs/s the wheel must rotate to cover a linear velocity of 167.5516 in/s; we can just
    # divide the linear velocity by the circumference of the wheel to get the angular velocity in revs/s and convert to degrees/s:
    # 167.5516 in/s * (1 / 37.7 revs/in) = 4.444 revs/s, or 1600 deg/s.
    #
    # Performing the same chain of calculations for the right wheel will yield an angular velocity 2400 of deg/s.

    trackWidth = robot._properties.trackWidthInches
    wheelCircumference = robot.leftWheel._properties.diameterInches * 2.0 * pi

    leftWheelTurningRadius::Float32 = targetMotion.radiusInches - (trackWidth / 2.0)
    rightWheelTurningRadius::Float32 = targetMotion.radiusInches + (trackWidth / 2.0)

    leftWheelTangentialSpeed::Float32 = leftWheelTurningRadius * deg2rad(targetMotion.angularVelocityDegpS)
    rightWheelTangentialSpeed::Float32 = rightWheelTurningRadius * deg2rad(targetMotion.angularVelocityDegpS)

    leftWheelAngularVelocity::Float32 = leftWheelTangentialSpeed / wheelCircumference * 360.0
    rightWheelAngularVelocity::Float32 = rightWheelTangentialSpeed / wheelCircumference * 360.0

    setLeftWheelAngularVelocity(robot, leftWheelAngularVelocity)
    setRightWheelAngularVelocity(robot, rightWheelAngularVelocity)
end