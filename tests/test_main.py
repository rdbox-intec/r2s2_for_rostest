#!/usr/bin/env python
import os
import unittest
import nodes.integrationtest
from unittest import mock


class TestApiRecord(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)

    def test_valid_error_user(self):
        try:
            del os.environ['R2S2_USER']
        except Exception:
            pass
        with self.assertRaises(SystemExit) as cm:
            nodes.integrationtest.main()
        self.assertEqual(cm.exception.code, 1)

    def test_valid_error_pass(self):
        os.environ['R2S2_USER'] = 'test'
        try:
            del os.environ['R2S2_PASS']
        except Exception:
            pass
        with self.assertRaises(SystemExit) as cm:
            nodes.integrationtest.main()
        self.assertEqual(cm.exception.code, 1)

    def test_valid_error_url_cert1(self):
        pcs = {
            'reception_url': 'https://r2s2.rdbox.lan/rest/v1/job'
        }
        tests = [
            {
                "target": "judge",
                "method": "Equal",
                "expect": "SUCCESS",
                "description": "is robot reach the goal"
            }
        ]
        os.environ['R2S2_USER'] = 'test'
        os.environ['R2S2_PASS'] = 'test'
        try:
            del os.environ['R2S2_CERT']
        except Exception:
            pass
        with self.assertRaises(SystemExit) as cm:
            nodes.integrationtest.main(pcs, tests)
        self.assertEqual(cm.exception.code, 2)

    def test_valid_error_url_cert2(self):
        pcs = {
            'reception_url': 'http://r2s2.rdbox.lan/rest/v1/job'
        }
        tests = [
            {
                "target": "judge",
                "method": "Equal",
                "expect": "SUCCESS",
                "description": "is robot reach the goal"
            }
        ]
        os.environ['R2S2_USER'] = 'test'
        os.environ['R2S2_PASS'] = 'test'
        os.environ['R2S2_CERT'] = '/path/to/cert'
        with self.assertRaises(SystemExit) as cm:
            nodes.integrationtest.main(pcs, tests)
        self.assertEqual(cm.exception.code, 2)

    @mock.patch('nodes.integrationtest.IntegrationTest.run')
    def test_valid_success1(self, run_m):
        pcs = {
            'reception_url': 'https://r2s2.rdbox.lan/rest/v1/job'
        }
        tests = [
            {
                "target": "judge",
                "method": "Equal",
                "expect": "SUCCESS",
                "description": "is robot reach the goal"
            }
        ]
        os.environ['R2S2_USER'] = 'test'
        os.environ['R2S2_PASS'] = 'test'
        os.environ['R2S2_CERT'] = '/path/to/cert'
        ret = nodes.integrationtest.main(pcs, tests)
        self.assertTrue(ret)

    @mock.patch('nodes.integrationtest.IntegrationTest.run')
    def test_valid_success2(self, run_m):
        pcs = {
            'reception_url': 'http://r2s2.rdbox.lan/rest/v1/job'
        }
        tests = [
            {
                "target": "judge",
                "method": "Equal",
                "expect": "SUCCESS",
                "description": "is robot reach the goal"
            }
        ]
        os.environ['R2S2_USER'] = 'test'
        os.environ['R2S2_PASS'] = 'test'
        try:
            del os.environ['R2S2_CERT']
        except Exception:
            pass
        ret = nodes.integrationtest.main(pcs, tests)
        self.assertTrue(ret)
