include("robot.jl")
include("plant.jl")

function main()
    bot::Robot = Robot()
    timestep::Float32 = 1.0

    # simulateTurningCircle(bot, MotionArc(60, 200.0))
    simulateTurningCircle(bot, MotionArc(0, 10.0))

    updatePose(bot, timestep)
end

main()