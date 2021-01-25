#!/usr/bin/env python
import unittest
import nodes.integrationtest
from swagger_server.models.record_list import RecordList
from swagger_server.models.record import Record


class DummyClass(unittest.TestCase):
    """Now create your empty TestClass.
     Automatically add TestCase to this TestClass.
    """
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)


class TestTestBuilder(unittest.TestCase):
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

        r = Record(result='{"judge": "SUCCESS"}')
        rl = RecordList.from_dict([r.to_dict()])
        record = nodes.integrationtest.ApiRecord(rl)
        test_cases = nodes.integrationtest.TestCases(params)
        tb = nodes.integrationtest.TestBuilder(test_cases, record)
        tb.add_test_cases_to_class(DummyClass)
        import inspect
        dc = DummyClass()
        self.assertTrue(hasattr(dc, 'test_is_robot_reach_the_goal') and
                        inspect.ismethod(getattr(dc, 'test_is_robot_reach_the_goal')))
        dc.test_is_robot_reach_the_goal()
