import rclpy
from rclpy.node import Node
from lawnny5_interfaces.srv import GetGlobalSetting
from lawnny5_interfaces.srv import SetGlobalSetting
from lawnny5_interfaces.srv import GetGlobalSettings
from diagnostic_msgs.msg import KeyValue
import json

class GlobalSettingsServer(Node):

    def __init__(self):
        super().__init__('global_settings_server')

        self.settings = {}
        self.load_settings()

        self.settings_publisher = self.create_publisher(KeyValue, 'global_setting_updated', 10)
        self.get_setting_srv = self.create_service(GetGlobalSetting, 'get_setting', self.get_setting_callback)
        self.set_settings_srv = self.create_service(SetGlobalSetting, 'set_setting', self.set_setting_callback)
        self.get_settings_srv = self.create_service(GetGlobalSettings, 'get_settings', self.get_settings_callback)

    def get_settings_callback(self, request, response):
        response.value = json.dumps(self.settings)
        return response

    def get_setting_callback(self, request, response):
        response.value = self.settings[request.name] or ""
        return response

    def set_setting_callback(self, request, response):
        self.set_setting(request.name, request.value)

        kv_message = KeyValue()
        kv_message.key = request.name
        kv_message.value = request.value
        self.settings_publisher.publish(kv_message)

        return response

    def load_settings(self):
        try:
            with open('lawnny-settings.json', 'r') as openfile:
                self.settings = json.load(openfile)
        except:
            return

    def save_settings(self):
        try:
            with open('lawnny-settings.json', 'w') as outfile:
                outfile.write(json.dumps(self.settings))
        except:
            return

    def set_setting(self, name, value):
        self.settings[name] = value
        self.save_settings()

def main(args=None):
    rclpy.init(args=args)

    node = GlobalSettingsServer()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
