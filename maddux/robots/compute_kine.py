import numpy as np
from maddux.environment import Environment
from maddux.objects import Ball
from maddux.robots.predefined_robots import simple_human_arm, custom_arm


def compute_kine(theta1, theta2, theta3, theta4, goalInCam):
    # goalInCam: 4x4
    si = np.sin(0.0872665)
    co = np.cos(0.0872665)

    camInEndEffector = np.matrix([ [0, -1, 0, 0],[si, 0, -co, 0],[co, 0, si, 0],[0, 0, 0, 1]]) 

    goalInEndEffector = camInEndEffector*goalInCam



    custom = custom_arm(q=[theta1, theta2, theta3, theta4])
    endEffectorInBaseLink = custom.fkine()
    goalInBaseLink = endEffectorInBaseLink*goalInEndEffector

    X = goalInBaseLink[0][3]
    Y = goalInBaseLink[1][3]
    Z = goalInBaseLink[2][3]


    
    # Create a ball as our target
    ball = Ball(np.array([X, Y, Z]), 3, target=True)
    
    # Create our environment
    env = Environment([15.0, 15.0, 15.0], dynamic_objects=[ball],
                      robot=custom)
    
    # Run inverse kinematics to find a joint config that lets arm touch ball
    return custom.ikine(ball.position)
    