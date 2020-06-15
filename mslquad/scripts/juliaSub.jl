#! /home/kunal/julia/julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, PoseStamped
rostypegen()
using .geometry_msgs.msg

function poseCB(msg::PoseStamped, pose)
    # extract the pose
    pose = msg.pose
end

function loop(quadPoses)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        for pose in quadPoses
            print("x:", pose.position.x, " y:", pose.position.y, " z:", pose.position.z, "\n")
        end
        rossleep(loop_rate)
    end
end

function main()
    init_node("getPose0_jl")
    quadNames = ["rexquad0"]
    quadPoses = [Pose()]
    poseTopic = "/mavros/local_position/pose"
    quadSubs = [Subscriber{PoseStamped}(name*poseTopic, poseCB, (pose,), queue_size=10) for (name, pose) in zip(quadNames, quadPoses)]
    loop(quadPoses)
end

main()