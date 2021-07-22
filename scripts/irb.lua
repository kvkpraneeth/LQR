function sysCall_init()
    -- Take a few handles from the scene:
    simBase=sim.getObjectHandle(sim.handle_self)
    joint1Handle = sim.getObjectHandle("IRB140_joint1")
    joint2Handle = sim.getObjectHandle("IRB140_joint2")
    joint3Handle = sim.getObjectHandle("IRB140_joint3")
    joint4Handle = sim.getObjectHandle("IRB140_joint4")
    joint5Handle = sim.getObjectHandle("IRB140_joint5")
    joint6Handle = sim.getObjectHandle("IRB140_joint6")
    
    if simROS2 then
    
        sim.addLog(sim.verbosity_scriptinfos, "ROS2 Interface was found")
        
        joint1Topic="joint1_control"
        joint2Topic="joint2_control"
        joint3Topic="joint3_control"
        joint4Topic="joint4_control"
        joint5Topic="joint5_control"
        joint6Topic="joint6_control"

        simTimeTopic="simTime"
        stateTopic="abbState"
        
        simTimePub=simROS2.createPublisher('/'..simTimeTopic, 'std_msgs/msg/Float32')
        statePub=simROS2.createPublisher('/'..stateTopic, 'std_msgs/msg/Float32MultiArray')
        
        joint1Sub=simROS2.createSubscription('/'..joint1Topic, 'std_msgs/msg/Float32', 'setJoint1Speed')
        joint2Sub=simROS2.createSubscription('/'..joint2Topic, 'std_msgs/msg/Float32', 'setJoint2Speed')
        joint3Sub=simROS2.createSubscription('/'..joint3Topic, 'std_msgs/msg/Float32', 'setJoint3Speed')
        joint4Sub=simROS2.createSubscription('/'..joint4Topic, 'std_msgs/msg/Float32', 'setJoint4Speed')
        joint5Sub=simROS2.createSubscription('/'..joint5Topic, 'std_msgs/msg/Float32', 'setJoint5Speed')
        joint6Sub=simROS2.createSubscription('/'..joint6Topic, 'std_msgs/msg/Float32', 'setJoint6Speed')
   
    else      
        sim.addLog(sim.verbosity_scriptinfos, "ROS2 Interface ain't here lmao")
    
    end
    
end

function setJoint1Speed(msg)
    sim.setJointTargetVelocity(joint1Handle, msg.data)
end
function setJoint2Speed(msg)
    sim.setJointTargetVelocity(joint2Handle, msg.data)
end
function setJoint3Speed(msg)
    sim.setJointTargetVelocity(joint3Handle, msg.data)
end
function setJoint4Speed(msg)
    sim.setJointTargetVelocity(joint4Handle, msg.data)
end
function setJoint5Speed(msg)
    sim.setJointTargetVelocity(joint5Handle, msg.data)
end
function setJoint6Speed(msg)
    sim.setJointTargetVelocity(joint6Handle, msg.data)
end

function getJointStates()
    
    joint1State=sim.getJointPosition(joint1Handle)
    joint2State=sim.getJointPosition(joint2Handle)
    joint3State=sim.getJointPosition(joint3Handle)
    joint4State=sim.getJointPosition(joint4Handle)
    joint5State=sim.getJointPosition(joint5Handle)
    joint6State=sim.getJointPosition(joint6Handle)

    array={joint1State, joint2State, joint3State, joint4State, joint5State, joint6State}
    msg={}
    msg.data=array
    return msg

end

function sysCall_actuation()

    if simROS2 then
        simROS2.publish(statePub, getJointStates())
        simROS2.publish(simTimePub, {data=sim.getSimulationTime()})
    end

end

function sysCall_cleanup()
    if simROS2 then
    
        simROS2.shutdownPublisher(simTimePub)
        simROS2.shutdownPublisher(statePub)
        
        simROS2.shutdownSubscription(joint1Sub)
        simROS2.shutdownSubscription(joint2Sub)
        simROS2.shutdownSubscription(joint3Sub)
        simROS2.shutdownSubscription(joint4Sub)
        simROS2.shutdownSubscription(joint5Sub)
        simROS2.shutdownSubscription(joint6Sub)
        
    end
end
