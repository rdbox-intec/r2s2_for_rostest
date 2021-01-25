#!/usr/bin/env python
import unittest
from nodes.integrationtest import TestCases


class TestTestCases(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)

    def test_constructor(self):
        params = [
            {
                "target": "judge",
                "method": "Equal",
                "expect": "SUCCESS",
                "description": "is robot reach the goal"
            }
        ]
        tcs = TestCases(params)
        target = len(tcs)
        self.assertEqual(target, 1)
        target = str(tcs)
        self.assertEqual(target, 'TestCases(TestCase(target: judge, method: Equal, expect: SUCCESS, description: test_is_robot_reach_the_goal))')
        target = tcs[0].get_target()
        self.assertEqual(target, 'judge')
