from modules.common_msgs.localization_msgs.pose_pb2 import Pose


class SpawnObjectParam:
	def __init__(self):
		self.type = None
		self.id = None
		self.attach_to = None
		self.random_pose = None
		self.transform = Pose()
		self.attributes = []
		self.initial_velocity = {}
		self.initial_angular_velocity = {}
		self.initial_control_command = {}
class KeyValue:
	def __init__(self, key, value):
		self.key = key
		self.value = value
		