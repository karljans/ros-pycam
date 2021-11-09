import rospy

PREFIX = 'PYCAM'

class PyCamLog:

    @staticmethod
    def debug(msg):
        rospy.logdebug(f"{PREFIX}: {msg}")

    @staticmethod
    def info(msg):
        rospy.loginfo(f"{PREFIX}: {msg}")

    @staticmethod
    def warn(msg):
        rospy.logwarn(f"{PREFIX}: {msg}")

    @staticmethod
    def error(msg):
        rospy.logerr(f"{PREFIX}: {msg}")

    @staticmethod
    def fatal(msg):
        rospy.logfatal(f"{PREFIX}: {msg}")