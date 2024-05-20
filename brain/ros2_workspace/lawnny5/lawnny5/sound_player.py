import rclpy
from rclpy.node import Node
import subprocess, os, time
from mpyg321.MPyg321Player import MPyg321Player
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class SoundPlayer(Node):

    def __init__(self):
        super().__init__('sound_player')

        self.bt_adapter_addr = '8C:88:0B:4A:27:A8'
        self.bt_speaker_addr = '12:11:CE:D4:E8:3A'
        self.mp3_player = None
        self.bt_sound_system_proc = None

        self.sound_dir = get_package_share_directory('lawnny5') + '/sounds/'

        self.create_subscription(
            String,
            'sound/play_by_name',
            self.handle_play_by_name,
            1)

        self.startup()

    def startup(self):
        self.connect_bluetooth_speaker(self.bt_adapter_addr, self.bt_speaker_addr)
        self.mp3_player = MPyg321Player()

    def shutdown(self):
        self.disconnect_bluetooth_speaker(self.bt_speaker_addr)

    def handle_play_by_name(self, msg):
        self.play_sound_by_name(msg.data)

    def play_sound_by_name(self, name):
        self.play_sound_file(self.sound_dir + name + '.mp3')

    def play_sound_file(self, file):
        self.get_logger().info("Playing sound: %s" % file)
        self.mp3_player.play_song(file)

    def disconnect_bluetooth_speaker(self, audio_device_addr):
        subprocess.run(['bluetoothctl', 'disconnect', audio_device_addr])

        if self.bt_sound_system_proc:
            self.bt_sound_system_proc.kill()
            self.bt_sound_system_proc = None

    def connect_bluetooth_speaker(self, adapter_addr, audio_device_addr):
        self.get_logger().info("Starting sound process...")
        # Startup sound subsystem
        env_vars = os.environ.copy()
        env_vars["AUDIO_DEVICE_ADDR"] = audio_device_addr
        self.bt_sound_system_proc = subprocess.Popen(['$LAWNNY5_ROOT/initialize_sound.sh'],
                                                     stdout=subprocess.PIPE,
                                                     stderr=subprocess.STDOUT,
                                                     env=env_vars, shell=True, executable="/bin/bash")

        time.sleep(1.0)

        self.get_logger().info("Connecting BT Device")

        subprocess.run(['bluetoothctl', 'select', adapter_addr])
        subprocess.run(['bluetoothctl', '--timeout=10', 'scan', 'on'])
        subprocess.run(['bluetoothctl', 'pair', audio_device_addr])
        subprocess.run(['bluetoothctl', 'trust', audio_device_addr])
        subprocess.run(['bluetoothctl', 'connect', audio_device_addr])


def main(args=None):
    rclpy.init(args=args)

    node = SoundPlayer()

    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
