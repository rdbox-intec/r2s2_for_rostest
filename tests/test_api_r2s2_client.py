#!/usr/bin/env python
from swagger_server.models.record_list import RecordList
import unittest
from nodes.integrationtest import ApiR2S2Client, ApiRecord
from unittest import mock
from swagger_server.models.job import Job
from swagger_server.models.api_response import ApiResponse


class TestApiR2S2Client(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)

    def test_build_config_http(self):
        api = ApiR2S2Client('http://test.com/v1', 'admin', 'pass')
        target = api.conf.ssl_ca_cert
        self.assertIsNone(target)
        target = api.recept_id
        self.assertEqual(target, -1)

    def test_build_config_https(self):
        api = ApiR2S2Client('https://test.com/v1', 'admin', 'pass', '/etc/secret/tls.crt')
        target = api.conf.ssl_ca_cert
        self.assertEqual(target, '/etc/secret/tls.crt')
        target = api.conf.assert_hostname
        self.assertEqual(target, False)

    @mock.patch('swagger_client.api.job_api.JobApi.put_job_with_http_info',
                return_value=(ApiResponse(id=7437), 200, None))
    def test_push_job_success200(self, mock_put):
        api = ApiR2S2Client('http://test.com/v1', 'admin', 'pass')
        api.push_job(Job())
        target = api.recept_id
        self.assertEqual(target, 7437)

    @mock.patch('swagger_client.api.job_api.JobApi.put_job_with_http_info',
                return_value=(ApiResponse(id=7437), 400, None))
    def test_push_job_fail400(self, mock_put):
        api = ApiR2S2Client('http://test.com/v1', 'admin', 'pass')
        with self.assertRaises(RuntimeError):
            api.push_job(Job())

    def test_search_job_not_put_yet(self):
        api = ApiR2S2Client('http://test.com/v1', 'admin', 'pass')
        with self.assertRaises(ValueError):
            api.search_job()

    @mock.patch('swagger_client.api.job_api.JobApi.put_job_with_http_info',
                return_value=(ApiResponse(id=7437), 200, None))
    @mock.patch('swagger_client.api.job_api.JobApi.find_job_with_http_info',
                return_value=([{"status": "DONE"}], 200, None))
    def test_search_job_success200(self, mock_put, mock_get):
        api = ApiR2S2Client('http://test.com/v1', 'admin', 'pass')
        api.push_job(Job())
        api_record = api.search_job()
        target = api_record.is_done()
        self.assertTrue(target)

    @mock.patch('swagger_client.api.job_api.JobApi.put_job_with_http_info',
                return_value=(ApiResponse(id=7437), 200, None))
    @mock.patch('swagger_client.api.job_api.JobApi.find_job_with_http_info',
                return_value=([{"status": "ERROR"}], 500, None))
    def test_search_job_fail500(self, mock_put, mock_get):
        api = ApiR2S2Client('http://test.com/v1', 'admin', 'pass')
        api.push_job(Job())
        with self.assertRaises(RuntimeError):
            api.search_job()
