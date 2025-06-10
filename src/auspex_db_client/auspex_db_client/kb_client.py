#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from auspex_msgs.srv import ExistsKnowledge, InsertKnowledge, QueryKnowledge, UpdateKnowledge, WriteKnowledge
from rclpy.executors import MultiThreadedExecutor


class KB_Client(Node):
    def __init__(self, node_name_prefix=''):
        super().__init__(node_name_prefix + 'knowledge_base_client')
        self._mte = MultiThreadedExecutor()
        self._exists_client = self.create_client(ExistsKnowledge, '/exists_knowledge')
        self._insert_client = self.create_client(InsertKnowledge, '/insert_knowledge')
        self._query_client = self.create_client(QueryKnowledge, '/query_knowledge')
        self._update_client = self.create_client(UpdateKnowledge, '/update_knowledge')
        self._write_client = self.create_client(WriteKnowledge, '/write_knowledge')

    def build_json_path(self, field, key, value):
        json_path = ''

        if not field and not key and not value:
            json_path = '$'

        elif field and not key and not value:
            json_path = '$[*].' + field

        elif field and key and value:
            json_path = '$[?(@.' + key + '=="' + value + '")].' + field

        elif field and not key and value:
            json_path = '$[?(@[*]=="' + value + '")].' + field

        elif not field and not key and value:
            json_path = '$[?(@[*]=="' + value + '")]'

        elif not field and key and value:
            json_path = '$[?(@.' + key + '=="' + value + '")]'

        else:
            json_path = '$'

        return json_path

    def query(self, collection, field='', key='', value=''):
        while not self._query_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for query client to come online...')

        query_request = QueryKnowledge.Request()
        query_request.collection = collection
        query_request.path = self.build_json_path(field, key, value)
        future = self._query_client.call_async(query_request)
        rclpy.spin_until_future_complete(self, future, executor=self._mte)
        if future.result() is None:
            return []
        answer = future.result().answer
        if answer:
            result = []
            for item in answer:
                try:
                    item = item.replace("'", '"')
                    item_dict = json.loads(item)
                except (json.JSONDecodeError, TypeError):
                    if not field:
                        result.append({'value':item})
                    else:
                        result.append({field:item})
                else:
                    result.append(item_dict)
            if len(result) == 1 and isinstance(result[0], list):
                return result[0]
            else:
                return result
        else:
            return []

    def write(self, collection, entity:str, field='', key='', value=''):
        while not self._write_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for write client to come online...')
        try:
            write_request = WriteKnowledge.Request()
            write_request.collection = collection
            if '.' in field:
                parent_field, actual_field = field.split('.', 1)            # The part after the last dot
                write_request.path = '$.' + parent_field + '[?(@.' + key + '=="' + value + '")].' + actual_field
            else:
                write_request.path =  self.build_json_path(field, key, value)
            write_request.entity = entity
            future = self._write_client.call_async(write_request)
            rclpy.spin_until_future_complete(self, future, executor=self._mte)
            if future.result() is None:
                return False
            return future.result().success
        except Exception as e:
            self.get_logger().error(f'Write request failed: {e}')
            return False

    def insert(self, collection, entity):
        while not self._insert_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for insert client to come online...')
        insert_request = InsertKnowledge.Request()
        insert_request.collection = collection
        insert_request.path = '$'
        insert_request.entity = entity
        future = self._insert_client.call_async(insert_request)
        rclpy.spin_until_future_complete(self, future, executor=self._mte)
        if future.result() is None:
            return False
        return future.result().success

    def update(self, collection, new_value, field='', key='', value=''):
        while not self._update_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for update client to come online...')
        update_request = UpdateKnowledge.Request()
        update_request.collection = collection
        update_request.path = self.build_json_path(field, key, value)
        update_request.value = new_value
        future = self._update_client.call_async(update_request)
        rclpy.spin_until_future_complete(self, future, executor=self._mte)
        if future.result() is None:
            return False
        return future.result().success

    def update_action_status(self, plan_id, action_id, new_status):
        while not self._update_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for update client to come online...')

        update_request = UpdateKnowledge.Request()
        update_request.collection = 'plan'

        # Create a nested JSON path to target a specific action by ID within a plan for a platform
        update_request.path = f'$[?(@.plan_id=="{str(plan_id)}")].actions[?(@.id=="{str(action_id)}")].status'
        update_request.value = new_status

        future = self._update_client.call_async(update_request)
        rclpy.spin_until_future_complete(self, future, executor=self._mte)

        if future.result() is None:
            return False
        return future.result().success
