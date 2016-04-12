from arm_test import arm_test
from animation_test import ball_animation_test, arm_animation_test
from plot_arm_test import plot_arm_test
from plot_test import plot_test

tests = {
    'arm': arm_test,
    'ball_animation': ball_animation_test,
    'arm_animation': arm_animation_test,
    'plot_arm': plot_arm_test,
    'plot': plot_test,
}

def run_test(test):
    if test in tests:
        tests[test]()
        return True
    return False
    
