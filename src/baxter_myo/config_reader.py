import sys
import ConfigParser

import rospkg

class ConfigReader(object):

    def __init__(self, file):
        """
        `file`: string
        """
        self._rp = rospkg.RosPack()
        self._config_path = self._rp.get_path('baxter_myo') \
                            + '/config/' + file
        self._config = ConfigParser.ConfigParser()
        self._config.read(self._config_path)

        self.left_arm = False
        self.right_arm = False
        self.push_thresh = 0

        self.left_angles = {}
        self.right_angles = {}

    def get_sections(self):
        return self._config.sections()

    def parse_general(self):
        """
        Returns True if parsing is succesfull, False otherwise.
        """
        try:
            self.left_arm = self._config.getboolean("GENERAL",
                                                    "left_arm")
            self.right_arm = self._config.getboolean("GENERAL",
                                                     "right_arm")
            self.push_thresh = self._config.getint("GENERAL",
                                                   "push_thresh")
        except ConfigParser.NoOptionError, err:
            print str(err)
            return False
        return True

    def parse_starting_pose(self):
        """
        Returns True if parsing is succesfull, False otherwise.
        """
        try:
            self.left_angles = eval(self._config.get("STARTING_POSE",
                                                     "left"))
            self.right_angles = eval(self._config.get("STARTING_POSE",
                                                      "right"))
        except ConfigParser.NoOptionError, err:
            print str(err)
            return False
        if not (type(self.right_angles) is dict and
                type(self.left_angles) is dict):
            print "Wrong var type in STARTING_POSE"
            return False
        return True

    def parse_all(self):
        results = [self.parse_general(),
                   self.parse_starting_pose()]
        if not all(results):
            print "Error while parsing. Exiting!"
            sys.exit(1)


def main():
    c = ConfigReader("demo_config")
    c.parse_all()
    print c.right_angles
    print c.left_angles
    print c.left_arm
    print c.push_thresh

if __name__ == "__main__":
    main()
