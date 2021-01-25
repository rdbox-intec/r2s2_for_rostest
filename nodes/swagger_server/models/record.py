# coding: utf-8

from __future__ import absolute_import
from datetime import date, datetime  # noqa: F401

from typing import List, Dict  # noqa: F401

from swagger_server.models.base_model_ import Model
from swagger_server import util
from swagger_server.models.job import Job


class Record(Model):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    """

    def __init__(self, id: int=None, uid: str=None, mid: int=None, status: str=None, start_time: int=None, end_time: int=None, result: str=None, job: Job=None):  # noqa: E501
        """Record - a model defined in Swagger

        :param id: The id of this Record.  # noqa: E501
        :type id: int
        :param uid: The uid of this Record.  # noqa: E501
        :type uid: str
        :param mid: The mid of this Record.  # noqa: E501
        :type mid: int
        :param status: The status of this Record.  # noqa: E501
        :type status: str
        :param start_time: The start_time of this Record.  # noqa: E501
        :type start_time: int
        :param end_time: The end_time of this Record.  # noqa: E501
        :type end_time: int
        :param result: The result of this Record.  # noqa: E501
        :type result: str
        :param job: The job of this Record.  # noqa: E501
        :type job: Job
        """
        self.swagger_types = {
            'id': int,
            'uid': str,
            'mid': int,
            'status': str,
            'start_time': int,
            'end_time': int,
            'result': str,
            'job': Job
        }

        self.attribute_map = {
            'id': 'id',
            'uid': 'uid',
            'mid': 'mid',
            'status': 'status',
            'start_time': 'startTime',
            'end_time': 'endTime',
            'result': 'result',
            'job': 'job'
        }

        self._id = id
        self._uid = uid
        self._mid = mid
        self._status = status
        self._start_time = start_time
        self._end_time = end_time
        self._result = result
        self._job = job

    @classmethod
    def from_dict(cls, dikt) -> 'Record':
        """Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The Record of this Record.  # noqa: E501
        :rtype: Record
        """
        return util.deserialize_model(dikt, cls)

    @property
    def id(self) -> int:
        """Gets the id of this Record.


        :return: The id of this Record.
        :rtype: int
        """
        return self._id

    @id.setter
    def id(self, id: int):
        """Sets the id of this Record.


        :param id: The id of this Record.
        :type id: int
        """

        self._id = id

    @property
    def uid(self) -> str:
        """Gets the uid of this Record.


        :return: The uid of this Record.
        :rtype: str
        """
        return self._uid

    @uid.setter
    def uid(self, uid: str):
        """Sets the uid of this Record.


        :param uid: The uid of this Record.
        :type uid: str
        """

        self._uid = uid

    @property
    def mid(self) -> int:
        """Gets the mid of this Record.


        :return: The mid of this Record.
        :rtype: int
        """
        return self._mid

    @mid.setter
    def mid(self, mid: int):
        """Sets the mid of this Record.


        :param mid: The mid of this Record.
        :type mid: int
        """

        self._mid = mid

    @property
    def status(self) -> str:
        """Gets the status of this Record.


        :return: The status of this Record.
        :rtype: str
        """
        return self._status

    @status.setter
    def status(self, status: str):
        """Sets the status of this Record.


        :param status: The status of this Record.
        :type status: str
        """

        self._status = status

    @property
    def start_time(self) -> int:
        """Gets the start_time of this Record.


        :return: The start_time of this Record.
        :rtype: int
        """
        return self._start_time

    @start_time.setter
    def start_time(self, start_time: int):
        """Sets the start_time of this Record.


        :param start_time: The start_time of this Record.
        :type start_time: int
        """

        self._start_time = start_time

    @property
    def end_time(self) -> int:
        """Gets the end_time of this Record.


        :return: The end_time of this Record.
        :rtype: int
        """
        return self._end_time

    @end_time.setter
    def end_time(self, end_time: int):
        """Sets the end_time of this Record.


        :param end_time: The end_time of this Record.
        :type end_time: int
        """

        self._end_time = end_time

    @property
    def result(self) -> str:
        """Gets the result of this Record.


        :return: The result of this Record.
        :rtype: str
        """
        return self._result

    @result.setter
    def result(self, result: str):
        """Sets the result of this Record.


        :param result: The result of this Record.
        :type result: str
        """

        self._result = result

    @property
    def job(self) -> Job:
        """Gets the job of this Record.


        :return: The job of this Record.
        :rtype: Job
        """
        return self._job

    @job.setter
    def job(self, job: Job):
        """Sets the job of this Record.


        :param job: The job of this Record.
        :type job: Job
        """

        self._job = job
