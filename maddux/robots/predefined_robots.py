from link import Link
from arm import Arm
import numpy as np
from maddux.environment import Environment



def simple_human_arm(seg1_len, seg2_len, q0, base=None):
	"""Creates a simple human-like robotic arm with 7 links and 2 segments
	with the desired lengths and starting joint configuration

	:param seg1_len: The length of the first segment of the arm
	:type seg1_len: int

	:param seg2_len: The length of the second segment of the arm
	:type seg2_len: int

	:param q0: 1xN vector of the starting joint configuration
	:type q0: numpy.ndarray

	:param base: (Optional) (x, y, z) location of arm base
	:type base: numpy.ndarray or None

	:returns: 7 link, 2 segment "human" arm.
	:rtype: maddux.robot.Arm
	"""
	L1 = Link(0, 0, 0, 1.571)
	L2 = Link(0, 0, 0, -1.571)
	L3 = Link(0, seg1_len, 0, -1.571)
	L4 = Link(0, 0, seg2_len, -1.571)
	L5 = Link(0, 0, 0, 1.571)
	L6 = Link(0, 0, 0, 1.571)
	L7 = Link(0, 0, 0, 0)
	links = np.array([L1, L2, L3, L4, L5, L6, L7])

	robot = Arm(links, q0, 'simple_human_arm', 4, base)
	
	return robot

def temp_arm():
	# L1 = Link(0, 0, 0, 0, q_lim=np.array([0, 0]))
	# L2 = Link(0, 3.5, 0, 1.57, q_lim = np.array([-4.36/2, 4.36/2]))
	# L3 = Link(0, 0, 9.5, 0, q_lim = np.array([-2.44/2, 2.44/2]))
	# L4 = Link(0, 0, 9.5, 0, q_lim = np.array([-4.36/2, 4.36/2]))
	# L5 = Link(0, 0, 3, 0, q_lim = np.array([-4.36/2, 4.36/2]))
	# links = np.array([L1, L2, L3, L4, L5])
	# q0 = [0, -4.36/2, -2.44/2, -4.36/2, 0]
	# q01 = [0, 0 ,0, 1.57,0]
	# q2 = [0,1.571,1.571,-1.2,1.571]

	# robot = Arm(links, np.array(q01), 'Custom', 5, None)
	L2 = Link(0, 3.5, 1.7, 1.571, q_lim = np.array([-4.36/2, 4.36/2]))
	L3 = Link(0, 0, 9.5, 0, q_lim = np.array([-2.44/2, 2.44/2]))
	L4 = Link(0, 0, 9.5, 0, q_lim = np.array([-4.36/2, 4.36/2]))
	L5 = Link(0, 0, 3, 0, q_lim = np.array([-4.36/2, 4.36/2]))
	links = np.array([L2, L3, L4, L5])
	q0 = [0, 0, 0, 0, 0]
	q01 = [1.571,0,0,0,0]
	q02 = [0, 0.698, -0.7854, 0.5]
	robot = Arm(links, np.array(q02), 'Custom', 4, None)

	env = Environment([25, 25, 25], robot=robot)

	# env = Environment([15.0, 15.0, 15.0], robot=robot)
	env.animate()
	return robot

def custom_arm(q=None):
	# theta, offset, length, twist
	# L1 = Link(0, 0, 0, 0, q_lim=np.array([0, 0]))
	L2 = Link(0, 3.5, 1.7, 1.571, q_lim = np.array([-4.36/2, 4.36/2]))
	L3 = Link(0, 0, 9.5, 0, q_lim = np.array([-2.44/2, 2.44/2]))
	L4 = Link(0, 0, 9.5, 0, q_lim = np.array([-4.36/2, 4.36/2]))
	L5 = Link(0, 0, 3, 0, q_lim = np.array([-4.36/2, 4.36/2]))
	links = np.array([L2, L3, L4, L5])
	q0 = [0, 0, 0, 0, 0]
	q01 = [1.571,0,0,0,0]
	if q is None:
		q02 = [0, 0.698, -0.7854, 0.5]
	else: q02 = q
	robot = Arm(links, np.array(q02), 'Custom', 4, None)

	return robot

def noodle_arm(seg_lens, q0, base=None):
	"""Creates a complex arm with 10 segments

	:param seg_lens: 1x10 vector of lengths of each sement
	:type seg_lens: numpy.ndarray

	:param q0: 1xN vector of the starting joint configuration
	:type q0: numpy.ndarray

	:param base: (Optional) optional (x, y, z) base location of arm
	:type base: numpy.ndarray or None

	:returns: "Noodle" arm
	:rtype: maddux.robot.Arm
	"""
	L1 = Link(0, seg_lens[0], 0, 1.571)
	L2 = Link(0, seg_lens[1], 0, -1.571)
	L3 = Link(0, seg_lens[2], 0, -1.571)
	L4 = Link(0, seg_lens[3], 0, 1.571)
	L5 = Link(0, seg_lens[4], 0, 1.571)
	L6 = Link(0, seg_lens[5], 0, -1.571)
	L7 = Link(0, seg_lens[6], 0, 1.571)
	L8 = Link(0, seg_lens[7], 0, 1.571)
	L9 = Link(0, seg_lens[8], 0, -1.571)
	L10 = Link(0, seg_lens[9], 0, 1.571)
	links = np.array([L1, L2, L3, L4, L5, L6, L7, L8, L9, L10])

	robot = Arm(links, q0, 'noodle_arm', 10, base)
	return robot
