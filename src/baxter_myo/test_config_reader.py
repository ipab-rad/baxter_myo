import unittest

from config_reader import ConfigReader


class TestConfigReader(unittest.TestCase):

    def setUp(self):
        self.cr_ = ConfigReader("test_config")
        self.cr_.parse_all()

    def test_left_angles(self):
        self.assertEqual(self.cr_.left_angles,
                         {'left_e0': 0.01, 'left_e1': 0.02,
                          'left_s0': 0.03, 'left_s1': 0.04,
                          'left_w0': 0.05, 'left_w1': 0.06,
                          'left_w2': 0.07})

    def test_right_angles(self):
        self.assertEqual(self.cr_.right_angles,
                         {'right_e0': 0.01, 'right_e1': 0.02,
                          'right_s0': 0.03, 'right_s1': 0.04,
                          'right_w0': 0.05, 'right_w1': 0.06,
                          'right_w2': 0.07})

    def test_left_arm(self):
        self.assertTrue(self.cr_.left_arm)

    def test_right_arm(self):
        self.assertTrue(self.cr_.right_arm)

    def test_push_thresh(self):
        self.assertEqual(self.cr_.push_thresh, 100)

    def test_mode(self):
        self.assertEqual(self.cr_.mode, "test_mode")


if __name__ == "__main__":
    unittest.main()
