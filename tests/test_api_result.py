#!/usr/bin/env python
import unittest
from nodes.integrationtest import ApiResult


class TestApiResult(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)

    def test_constructor_error(self):
        api = ApiResult('hoge')
        target = api.get_by_dot('A')
        self.assertEqual(target, None)

    def test_value(self):
        api = ApiResult('{"A": 1}')
        target = api.get_by_dot('A')
        self.assertEqual(target, 1)

    def test_str(self):
        api = ApiResult('{"A": "B"}')
        target = api.get_by_dot('A')
        self.assertEqual(target, 'B')

    def test_bool(self):
        api = ApiResult('{"A": true}')
        target = api.get_by_dot('A')
        self.assertEqual(target, True)

    def test_2nestes(self):
        api = ApiResult('{"A": {"B": 1}}')
        target = api.get_by_dot('A.B')
        self.assertEqual(target, 1)
        target = api.get_by_dot('A')
        self.assertDictEqual(target, {"B": 1})

    def test_notfound(self):
        api = ApiResult('{"A": {"B": 1}}')
        target = api.get_by_dot('A.C')
        self.assertEqual(target, None)

    def test_3nestes(self):
        api = ApiResult('{"A": {"B": {"c": false}}}')
        target = api.get_by_dot('A.B.c')
        self.assertEqual(target, False)
