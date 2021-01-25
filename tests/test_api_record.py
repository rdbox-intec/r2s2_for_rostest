#!/usr/bin/env python
import unittest
from nodes.integrationtest import ApiRecord
from swagger_server.models.record_list import RecordList
from swagger_server.models.record import Record


class TestApiRecord(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)

    def test_constructor_error(self):
        with self.assertRaises(ValueError):
            ApiRecord(RecordList.from_dict([{}, {}]))

    def test_done_true(self):
        record = Record(status='DONE')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.is_done()
        self.assertEqual(target, True)

    def test_done_false(self):
        record = Record(status='PENDING')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.is_done()
        self.assertEqual(target, False)

    def test_value(self):
        record = Record(result='{"A": 1}')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.get_result_by('A')
        self.assertEqual(target, 1)

    def test_str(self):
        record = Record(result='{"A": "B"}')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.get_result_by('A')
        self.assertEqual(target, 'B')

    def test_bool(self):
        record = Record(result='{"A": true}')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.get_result_by('A')
        self.assertEqual(target, True)

    def test_2nestes(self):
        record = Record(result='{"A": {"B": 1}}')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.get_result_by('A.B')
        self.assertEqual(target, 1)
        target = api.get_result_by('A')
        self.assertDictEqual(target, {"B": 1})

    def test_notfound(self):
        record = Record(result='{"A": {"B": 1}}')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.get_result_by('A.C')
        self.assertEqual(target, None)

    def test_3nestes(self):
        record = Record(result='{"A": {"B": {"c": false}}}')
        record_list = RecordList.from_dict([record.to_dict()])
        api = ApiRecord(record_list)
        target = api.get_result_by('A.B.c')
        self.assertEqual(target, False)
