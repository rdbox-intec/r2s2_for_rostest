# coding: utf-8

from __future__ import absolute_import
from datetime import date, datetime  # noqa: F401

from typing import List, Dict  # noqa: F401

from swagger_server.models.base_model_ import Model
from swagger_server import util


class Result(Model):
    """NOTE: This class is auto generated by the swagger code generator program.

    Do not edit the class manually.
    """

    def __init__(self, id: int=None, uid: str=None, mid: int=None, status: str=None, start_time: int=None, end_time: int=None, result: str=None, namespace: str=None):  # noqa: E501
        """Result - a model defined in Swagger

        :param id: The id of this Result.  # noqa: E501
        :type id: int
        :param uid: The uid of this Result.  # noqa: E501
        :type uid: str
        :param mid: The mid of this Result.  # noqa: E501
        :type mid: int
        :param status: The status of this Result.  # noqa: E501
        :type status: str
        :param start_time: The start_time of this Result.  # noqa: E501
        :type start_time: int
        :param end_time: The end_time of this Result.  # noqa: E501
        :type end_time: int
        :param result: The result of this Result.  # noqa: E501
        :type result: str
        :param namespace: The namespace of this Result.  # noqa: E501
        :type namespace: str
        """
        self.swagger_types = {
            'id': int,
            'uid': str,
            'mid': int,
            'status': str,
            'start_time': int,
            'end_time': int,
            'result': str,
            'namespace': str
        }

        self.attribute_map = {
            'id': 'id',
            'uid': 'uid',
            'mid': 'mid',
            'status': 'status',
            'start_time': 'startTime',
            'end_time': 'endTime',
            'result': 'result',
            'namespace': 'namespace'
        }

        self._id = id
        self._uid = uid
        self._mid = mid
        self._status = status
        self._start_time = start_time
        self._end_time = end_time
        self._result = result
        self._namespace = namespace

    @classmethod
    def from_dict(cls, dikt) -> 'Result':
        """Returns the dict as a model

        :param dikt: A dict.
        :type: dict
        :return: The Result of this Result.  # noqa: E501
        :rtype: Result
        """
        return util.deserialize_model(dikt, cls)

    @property
    def id(self) -> int:
        """Gets the id of this Result.


        :return: The id of this Result.
        :rtype: int
        """
        return self._id

    @id.setter
    def id(self, id: int):
        """Sets the id of this Result.


        :param id: The id of this Result.
        :type id: int
        """

        self._id = id

    @property
    def uid(self) -> str:
        """Gets the uid of this Result.


        :return: The uid of this Result.
        :rtype: str
        """
        return self._uid

    @uid.setter
    def uid(self, uid: str):
        """Sets the uid of this Result.


        :param uid: The uid of this Result.
        :type uid: str
        """

        self._uid = uid

    @property
    def mid(self) -> int:
        """Gets the mid of this Result.


        :return: The mid of this Result.
        :rtype: int
        """
        return self._mid

    @mid.setter
    def mid(self, mid: int):
        """Sets the mid of this Result.


        :param mid: The mid of this Result.
        :type mid: int
        """

        self._mid = mid

    @property
    def status(self) -> str:
        """Gets the status of this Result.


        :return: The status of this Result.
        :rtype: str
        """
        return self._status

    @status.setter
    def status(self, status: str):
        """Sets the status of this Result.


        :param status: The status of this Result.
        :type status: str
        """

        self._status = status

    @property
    def start_time(self) -> int:
        """Gets the start_time of this Result.


        :return: The start_time of this Result.
        :rtype: int
        """
        return self._start_time

    @start_time.setter
    def start_time(self, start_time: int):
        """Sets the start_time of this Result.


        :param start_time: The start_time of this Result.
        :type start_time: int
        """

        self._start_time = start_time

    @property
    def end_time(self) -> int:
        """Gets the end_time of this Result.


        :return: The end_time of this Result.
        :rtype: int
        """
        return self._end_time

    @end_time.setter
    def end_time(self, end_time: int):
        """Sets the end_time of this Result.


        :param end_time: The end_time of this Result.
        :type end_time: int
        """

        self._end_time = end_time

    @property
    def result(self) -> str:
        """Gets the result of this Result.


        :return: The result of this Result.
        :rtype: str
        """
        return self._result

    @result.setter
    def result(self, result: str):
        """Sets the result of this Result.


        :param result: The result of this Result.
        :type result: str
        """

        self._result = result

    @property
    def namespace(self) -> str:
        """Gets the namespace of this Result.


        :return: The namespace of this Result.
        :rtype: str
        """
        return self._namespace

    @namespace.setter
    def namespace(self, namespace: str):
        """Sets the namespace of this Result.


        :param namespace: The namespace of this Result.
        :type namespace: str
        """

        self._namespace = namespace
