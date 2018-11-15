import numpy as np
from maddux.environment import Environment
from maddux.objects import Ball
from maddux.robots import simple_human_arm, custom_arm

def arm_animation():
    """Animate the arm moving to touch a ball"""

    # Declare a human arm
    q0 = np.array([0.5, 0.2, 0, 0.5, 1.5])
    human_arm = simple_human_arm(2.0, 2.0, q0, np.array([2.0, 2.0, 0.0]))
    custom = custom_arm()

    
    # Create a ball as our target
    ball = Ball(np.array([5.0, 5.0, 2.0]), 3, target=True)
    
    # Create our environment
    env = Environment([15.0, 15.0, 15.0], dynamic_objects=[ball],
                      robot=custom)
    
    # Run inverse kinematics to find a joint config that lets arm touch ball
    custom.ikine(ball.position)
    
    # Animate
    env.animate(5.0)

if __name__ == '__main__':
    arm_animation()
