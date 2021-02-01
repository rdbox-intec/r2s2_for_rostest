#!/usr/bin/env python
import unittest
import rostest
import rospy
import os
import sys
import types
import functools
import json
import re
import urllib
from typing import Any, Callable
from swagger_client.configuration import Configuration
from swagger_client.api_client import ApiClient
from swagger_client.api.job_api import JobApi
from swagger_client.api.internal_api import InternalApi
from swagger_server.models.job import Job
from swagger_server.models.record_list import RecordList
from swagger_server.models.record import Record
from swagger_server.models.api_response import ApiResponse


class TestContainer(unittest.TestCase):
    """Now create your empty TestClass.
     Automatically add TestCase to this TestClass.
    """
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)


class ApiResult(object):
    def __init__(self, result: str):
        try:
            self.d = json.loads(result)
        except Exception:
            rospy.loginfo('Not comming result record.')
            self.d = {}

    def get_by_dot(self, keys: str) -> Any:
        try:
            return self.__get(self.d, keys)
        except KeyError:
            rospy.logwarn('KeyError: {}'.format(keys))
            return None

    def __get(self, d: dict, keys: str) -> Any:
        if "." in keys:
            key, rest = keys.split(".", 1)
            return self.__get(d[key], rest)
        else:
            return d[keys]


class ApiRecord(object):
    def __init__(self, record_list: RecordList):
        if len(record_list) != 1:
            raise ValueError('The length of the RecordList must be 1.')
        self.record = Record.from_dict(record_list[0])
        self.result = ApiResult(self.record.result)

    def is_done(self) -> bool:
        return self.record.status.strip() == 'DONE'

    def get_result_by(self, keys: str) -> Any:
        return self.result.get_by_dot(keys)


class ApiR2S2Client(object):
    def __init__(self, url: str, user: str, pswd: str, cert: str = None):  # noqa: E501
        self.recept_id = -1
        self.conf = self.__build_config(url, user, pswd, cert)
        self.client = ApiClient(self.conf,
                                header_name="Authorization",
                                header_value=self.conf.get_basic_auth_token())
        # https://github.com/swagger-api/swagger-codegen/issues/6503
        self.client.rest_client.pool_manager.connection_pool_kw['retries'] = 10
        self.job_api = JobApi(self.client)
        self.internal_api = InternalApi(self.client)

    def push_job(self, job: Job) -> ApiResponse:
        # tuple(data, status, headers)
        data, status, _ = self.job_api.put_job_with_http_info(job)
        if status != 200:
            raise RuntimeError('Failed to put in a job.')
        else:
            self.recept_id = data.id
            return data

    def search_job(self) -> ApiRecord:
        if self.recept_id < 0:
            raise ValueError('You need submit a JOB.')
        # tuple(data, status, headers)
        data, status, _ = self.job_api.find_job_with_http_info(
            id=self.recept_id)
        if status != 200:
            raise RuntimeError('Failed to find the Job.')
        else:
            return ApiRecord(RecordList.from_dict(data))

    def __build_config(self, url: str, user: str, pswd: str, cert: str = None) -> Configuration:  # noqa: E501
        configuration = Configuration()
        configuration.host = url
        configuration.username = user
        configuration.password = pswd
        if cert is not None:
            configuration.ssl_ca_cert = cert
            configuration.assert_hostname = False
        return configuration


class TestCases(list):
    class TestCase(object):
        def __init__(self, target: str, method_name: str, expct, description=None):  # noqa: E501
            self.target = target
            self.method_name = method_name
            self.expct = expct
            if description is None:
                self.description = self.__make_template_description()
            else:
                self.description = self.__format_description(description)

        def __str__(self):
            return 'TestCase(target: {}, '\
                'method: {}, '\
                'expect: {}, '\
                'description: {})'.format(self.target,
                                          self.method_name,
                                          self.expct,
                                          self.description)

        def get_target(self) -> str:
            return self.target

        def get_description(self) -> str:
            return self.description

        def get_expect(self):
            return self.expct

        def get_method_name(self) -> str:
            return self.method_name

        def get_formal_method_name(self) -> str:
            return 'assert' + self.method_name

        def __make_template_description(self) -> str:
            return 'test_{}'.format('_'.join([str(self.target),
                                             str(self.method_name),
                                             re.sub(re.compile("[!-/:-@[-`{-~]"), '', str(self.expct))]))  # noqa: E501

        def __format_description(self, description: str) -> str:
            return 'test_{}'.format('_'.join(re.sub(re.compile("[!-/:-@[-`{-~]"), '', str(description)).split(' ')))  # noqa: E501

    def __init__(self, raw_params_tests: list):
        for test in raw_params_tests:
            test_case = TestCases.TestCase(test.get('target'),
                                           test.get('method'),
                                           test.get('expect'),
                                           test.get('description', None))
            self.append(test_case)

    def __str__(self):
        ret = ''
        if isinstance(self, (list, tuple)):
            ret = ', '.join(str(elem) for elem in self)
        else:
            pass  # handle other types here
        return 'TestCases({})'.format(ret)


class TestBuilder(object):
    def __init__(self, test_cases: TestCases, record: ApiRecord):  # noqa: E501
        self.test_cases = test_cases
        self.record = record

    def add_test_cases_to_class(self, target_class):
        for test_case in self.test_cases:
            method_name = test_case.get_formal_method_name()
            target = self.record.get_result_by(test_case.get_target())
            expct = test_case.get_expect()
            test_func = self.assemble_one_method(method_name, target, expct)
            test_name = test_case.get_description()
            setattr(target_class, test_name, test_func)

    def assemble_one_method(self, method_name: str, target: Any, expct: Any):
        def frame_method(self):
            do_something = getattr(self, method_name)
            do_something(target, expct)
        method = self.__copy_func(frame_method)
        return method

    def __copy_func(self, f: Callable):
        """Based on http://stackoverflow.com/a/6528148/190597 (Glenn Maynard)
        """
        g = types.FunctionType(f.__code__, f.__globals__, name=f.__name__,
                               argdefs=f.__defaults__,
                               closure=f.__closure__)
        g = functools.update_wrapper(g, f)
        g.__kwdefaults__ = f.__kwdefaults__
        return g


class IntegrationTest(object):
    def __init__(self, pcs: dict = None, tests: list = None):
        # Node
        if pcs is None and tests is None:
            rospy.init_node('integrationtest')
        # Param
        self.raw_params_pcs = self.__get_preconditions(pcs)
        self.raw_params_tests = self.__get_tests(tests)
        IntegrationTest.validate_cert_url(self.raw_params_pcs)
        self.test_cases = TestCases(self.raw_params_tests)
        self.apis_client = ApiR2S2Client(
            IntegrationTest.get_url(self.raw_params_pcs),
            IntegrationTest.get_user(),
            IntegrationTest.get_pass(),
            IntegrationTest.get_cert())

    def run(self):
        # push
        job = self.__build_job_instance()
        self.apis_client.push_job(job)
        # Loop
        record = ApiRecord(RecordList.from_dict([{}]))
        rate = rospy.Rate(0.2)
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and self.__within_the_time_limit(start):
            record = self.apis_client.search_job()
            if record.is_done():
                break
            print('wait...')
            rate.sleep()
        # Test
        if self.__within_the_time_limit(start):
            if record.is_done():
                self.__exec_rostest_success(record)
            else:
                self.__exec_rostest_failure(record)
        else:
            self.__exec_rostest_timeout(record)

    def __within_the_time_limit(self, start: float) -> int:
        limit = self.raw_params_pcs['timeout_sec']
        end = rospy.Time.now().to_sec()
        return end - start <= limit

    def __exec_rostest_success(self, record: ApiRecord):
        tb = TestBuilder(self.test_cases, record)
        tb.add_test_cases_to_class(TestContainer)
        rostest.rosrun("r2s2_for_rostest", 'integrationtest', TestContainer)

    def __exec_rostest_failure(self, record: ApiRecord):
        rospy.logerr('The status that returned incorrect result!!!!!!!!!!!')
        tb = TestBuilder(self.test_cases, record)
        tb.add_test_cases_to_class(TestContainer)
        rostest.rosrun("r2s2_for_rostest", 'integrationtest', TestContainer)
        raise SyntaxError()

    def __exec_rostest_timeout(self, record: ApiRecord):
        rospy.logerr('This TESTS has timed out!!!!!!!!!!!')
        tb = TestBuilder(self.test_cases, record)
        tb.add_test_cases_to_class(TestContainer)
        rostest.rosrun("r2s2_for_rostest", 'integrationtest', TestContainer)
        raise TimeoutError()

    def __build_job_instance(self) -> Job:
        job = Job.from_dict(self.raw_params_pcs)
        job.id = -1
        job.mid = -1
        job.unique_name = os.getenv('CI_JOB_STAGE', 'main')
        job.uid = IntegrationTest.get_user()
        return job

    def __get_preconditions(self, v: dict = None) -> dict:
        if v is None:
            return rospy.get_param('~preconditions', {})
        else:
            return v

    def __get_tests(self, v: list = None) -> list:
        if v is None:
            return rospy.get_param('~tests', [])
        else:
            return v

    @classmethod
    def get_user(cls) -> str:
        return os.environ['R2S2_USER']

    @classmethod
    def get_pass(cls) -> str:
        return os.environ['R2S2_PASS']

    @classmethod
    def get_cert(cls) -> str:
        return os.environ.get('R2S2_CERT', None)

    @classmethod
    def get_url(cls, pcs: dict) -> str:
        return os.environ.get('R2S2_URL',
                              pcs.get('reception_url',
                                      None))

    @classmethod
    def validate_cert_url(cls, pcs: dict):
        o = urllib.parse.urlparse(IntegrationTest.get_url(pcs))
        if o.scheme == 'http':
            if IntegrationTest.get_cert() is not None:
                raise AssertionError('When communicating with'
                                     'the http protocol, '
                                     'do not specify R2S2_CERT.')
        elif o.scheme == 'https':
            if IntegrationTest.get_cert() is None:
                raise AssertionError('When communicating with'
                                     'the https protocol, '
                                     'specify R2S2_CERT.')
        else:
            raise AssertionError('Invalid protocol')


def main(pcs: dict = None, tests: list = None):
    # Validate
    try:
        IntegrationTest.get_user()
        IntegrationTest.get_pass()
    except Exception:
        import traceback
        rospy.logerr(traceback.format_exc())
        rospy.logerr('Please set the environment variables correctly.')
        rospy.logerr('R2S2_USER, R2S2_PASS, and (R2S2_URL, R2S2_CERT)')
        sys.exit(1)
    # Main processing
    try:
        test_master = IntegrationTest(pcs, tests)
        test_master.run()
    except Exception:
        import traceback
        rospy.logerr(traceback.format_exc())
        rospy.logerr('An error has occurred during processing.')
        sys.exit(2)
    return True


if __name__ == '__main__':
    main()
    # Successful
    sys.exit(0)
