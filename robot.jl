struct _WheelProperties
    diameterInches::Int

    _WheelProperties() = new(6)
end

mutable struct Wheel
    _properties::_WheelProperties

    velocityDegpS::Float32

    Wheel() = new(_WheelProperties(), 0.0)
end

struct _RobotProperties
    trackWidthInches::Int
    maxWheelVelocityDegpS::Float32

    _RobotProperties() = new(24, (360 * 3))
end

struct Pose
    x::Float32
    y::Float32
    theta::Float32
end

mutable struct Robot
    _properties::_RobotProperties
    leftWheel::Wheel
    rightWheel::Wheel
    pose::Pose

    function Robot()
        return new(
            _RobotProperties(),
            Wheel(),
            Wheel(),
            Pose(0.0, 0.0, 0.0)
        )
    end
end

function setLeftWheelAngularVelocity(robot::Robot, velocityDegpS::Float32)
    # TODO: Cap velocity
    robot.leftWheel.velocityDegpS = velocityDegpS
end

function setRightWheelAngularVelocity(robot::Robot, velocityDegpS::Float32)
    # TODO: Cap velocity
    robot.rightWheel.velocityDegpS = velocityDegpS
end

function getLeftWheelAngularVelocity(robot::Robot)
    # TODO: Incorporate a noise model. If this were a sensor the reading would
    # not be perfectly true each time
    return robot.leftWheel.velocityDegpS
end

function getRightWheelAngularVelocity(robot::Robot)
    # TODO: Incorporate a noise model. If this were a sensor the reading would
    # not be perfectly true each time
    return robot.rightWheel.velocityDegpS
end

function updatePose(robot::Robot, dt::Float32)
    wheelCircumferenceInches = robot.leftWheel._properties.diameterInches * 2.0 * pi
    trackWidth = robot._properties.trackWidthInches

    # Units check:
    # angular velocity is assumed to be deg/s, want tangential velocity in inches/s
    # (<x> deg / 1 s) * (<y> inches / 1 rev) * (1 rev / 360 deg) = (x * y / 360) inches/s
    rightWheelLinearVel = getRightWheelAngularVelocity(robot) * wheelCircumferenceInches / 360.0
    leftWheelLinearVel = getLeftWheelAngularVelocity(robot) * wheelCircumferenceInches / 360.0

    # Compute the robot's angular velocity and rotation radius w.r.t.
    # the ICC. See http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
    # as a reference.
    # Radius is in inches
    curvatureRadius = trackWidth / 2.0 * ((rightWheelLinearVel + leftWheelLinearVel) / (rightWheelLinearVel - leftWheelLinearVel))
    # Vel is rad/s
    curvatureVel  = (rightWheelLinearVel - leftWheelLinearVel) / trackWidth

    # Compute the ICC position
    iccX = robot.pose.x - (curvatureRadius * sind(robot.pose.theta))
    iccY = robot.pose.y + (curvatureRadius * cosd(robot.pose.theta))

    iccRotation = [cos(curvatureVel * dt)   -sin(curvatureVel * dt) 0;
                   sin(curvatureVel * dt)   cos(curvatureVel * dt)  0;
                   0                        0                       1]
    poseAtZero = iccRotation * [(robot.pose.x - iccX);
                                (robot.pose.y - iccY);
                                deg2rad(robot.pose.theta)]
    poseTranslated = poseAtZero + [iccX;
                                   iccY;
                                   (curvatureVel * dt)]

    robot.pose = Pose(poseTranslated[1], poseTranslated[2], rad2deg(poseTranslated[3]))
end