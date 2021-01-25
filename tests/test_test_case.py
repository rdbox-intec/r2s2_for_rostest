#!/usr/bin/env python
import unittest
from nodes.integrationtest import TestCases


class TestTestCase(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)

    def test_description_template(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS')
        target = tc.get_description()
        self.assertTrue(target.startswith('test_'))
        self.assertEqual(target, 'test_judge_Equal_SUCCESS')

    def test_description_template_symbol(self):
        tc = TestCases.TestCase('sim_time_sec', 'Less', 50.785297)
        target = tc.get_description()
        self.assertTrue(target.startswith('test_'))
        self.assertEqual(target, 'test_sim_time_sec_Less_50785297')

    def test_format_description(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS', 'is robot reach the goal')
        target = tc.get_description()
        self.assertTrue(target.startswith('test_'))
        self.assertEqual(target, 'test_is_robot_reach_the_goal')

    def test_format_description_symbol(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS', 'is robot reach the goal!!!!')
        target = tc.get_description()
        self.assertTrue(target.startswith('test_'))
        self.assertEqual(target, 'test_is_robot_reach_the_goal')

    def test_get_target(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS', 'is robot reach the goal')
        target = tc.get_target()
        self.assertEqual(target, 'judge')

    def test_get_expect(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS', 'is robot reach the goal')
        target = tc.get_expect()
        self.assertEqual(target, 'SUCCESS')

    def test_get_method_name(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS', 'is robot reach the goal')
        target = tc.get_method_name()
        self.assertEqual(target, 'Equal')

    def test_get_formal_method_name(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS', 'is robot reach the goal')
        target = tc.get_formal_method_name()
        self.assertEqual(target, 'assertEqual')

    def test_str(self):
        tc = TestCases.TestCase('judge', 'Equal', 'SUCCESS')
        target = str(tc)
        self.assertEqual(target, 'TestCase(target: judge, method: Equal, expect: SUCCESS, description: test_judge_Equal_SUCCESS)')
