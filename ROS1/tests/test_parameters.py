"""Private ROS parameter validation, including ambiguous booleans and strings."""
import unittest
from unittest.mock import patch

try:
    from drone_sorting.interfaces import parameters
except ImportError:
    parameters = None


@unittest.skipIf(parameters is None, 'Requires ROS Noetic')
class ParameterTests(unittest.TestCase):
    def test_numeric_readers_reject_ambiguous_types(self):
        for reader, default in ((parameters.read_float, 1.0), (parameters.read_int, 1)):
            for value in (True, False, '1', None, [], {}):
                with self.subTest(reader=reader.__name__, value=value):
                    with patch.object(parameters.rospy, 'get_param', return_value=value):
                        with self.assertRaisesRegex(ValueError, '~limit'):
                            reader('limit', default)
        with patch.object(parameters.rospy, 'get_param', return_value=1.5):
            with self.assertRaises(ValueError):
                parameters.read_int('limit', 1)

    def test_float_and_integer_values_and_finite_limits(self):
        with patch.object(parameters.rospy, 'get_param', return_value=2):
            self.assertEqual(parameters.read_int('count', 1), 2)
            self.assertIsInstance(parameters.read_float('limit', 1), float)
        for value in (float('nan'), float('inf'), -float('inf')):
            with patch.object(parameters.rospy, 'get_param', return_value=value):
                with self.assertRaises(ValueError):
                    parameters.read_float('limit', 1)
        with patch.object(parameters.rospy, 'get_param', return_value=0):
            self.assertEqual(parameters.read_float('limit', 1), 0)
            with self.assertRaises(ValueError):
                parameters.read_float('limit', 1, positive=True)

    def test_boolean_requires_actual_boolean(self):
        for value in ('false', 0, 1, None):
            with patch.object(parameters.rospy, 'get_param', return_value=value):
                with self.assertRaises(ValueError):
                    parameters.read_bool('enabled', False)
        for value in (True, False):
            with patch.object(parameters.rospy, 'get_param', return_value=value):
                self.assertIs(parameters.read_bool('enabled', False), value)

    def test_optional_lists_validate_every_element(self):
        for value in ('1,2', [True], ['1'], [None], [float('nan')], [float('inf')]):
            with patch.object(parameters.rospy, 'get_param', return_value=value):
                with self.assertRaises(ValueError):
                    parameters.read_float_list('bounds', None)
        with patch.object(parameters.rospy, 'get_param', return_value=None):
            self.assertIsNone(parameters.read_float_list('bounds', None))
            with self.assertRaises(ValueError):
                parameters.read_float_list('bounds', [1])
        with patch.object(parameters.rospy, 'get_param', return_value=[1, -2, 0]):
            self.assertEqual(parameters.read_float_list('bounds', None), [1., -2., 0.])
