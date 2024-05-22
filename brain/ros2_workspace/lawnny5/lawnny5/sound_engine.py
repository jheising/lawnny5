import rclpy
from rclpy.node import Node
import subprocess, os, time
from mpyg321.MPyg123Player import MPyg123Player
from std_msgs.msg import String
import re
import hashlib
import tempfile
import requests
import copy
import json

ELEVEN_LABS_API_KEY = os.environ.get("ELEVEN_LABS_API_KEY")
LAWNNY5_CACHE = os.environ.get("LAWNNY5_CACHE")
TTS_CHUNK_SIZE = 1024
SPEECH_SETTINGS = {
    "model_id": "eleven_multilingual_v2",
    "voice_settings": {
        "stability": 0.35,
        "similarity_boost": 0.5,
        "style": 0.45,
        "use_speaker_boost": True
    }
}

class SoundEngine(Node):

    def __init__(self):
        super().__init__('sound_engine')

        self.bt_adapter_addr = '8C:88:0B:4A:27:A8'
        self.bt_speaker_addr = '12:11:CE:D4:E8:3A' # Hummingbird
        # self.bt_speaker_addr = "6C:47:60:9B:98:F5" # Sony
        self.mp3_player = None
        self.bt_sound_system_proc = None

        self.speech_settings_hash = hashlib.md5(json.dumps(SPEECH_SETTINGS, sort_keys=True).encode()).hexdigest()

        self.sound_dir = LAWNNY5_CACHE + '/sounds/'

        self.create_subscription(
            String,
            'sound/play_by_name',
            self.handle_play_by_name,
            1)

        self.create_subscription(
            String,
            'sound/speak_text',
            self.handle_speak_text,
            1)

        # Uncomment if you want Lawnny to play all chat sounds
        # self.create_subscription(
        #     String,
        #     'personality/chat_output',
        #     self.handle_speak_text,
        #     1)

        self.startup()

    def startup(self):
        self.connect_bluetooth_speaker(self.bt_adapter_addr, self.bt_speaker_addr)
        self.mp3_player = MPyg123Player()

    def shutdown(self):
        self.disconnect_bluetooth_speaker(self.bt_speaker_addr)

    def handle_play_by_name(self, msg):
        self.play_sound_by_name(msg.data)

    def handle_speak_text(self, msg):
        self.speak_text(msg.data)

    def play_sound_by_name(self, name):
        self.play_sound_file(self.sound_dir + name + '.mp3')

    def speak_text(self, speech, use_cache=True):
        text_fingerprint = hashlib.md5((re.sub(r'[^A-Za-z0-9]', '', speech).lower() + self.speech_settings_hash).encode()).hexdigest()

        if use_cache:
            # Remove all non-alphanumeric chars, lowercase, and md5 hash it to generate a hash key for the text
            filename = self.sound_dir + text_fingerprint + ".mp3"

            if os.path.exists(filename):
                self.play_sound_file(filename)
                return

        else:
            filename = tempfile.gettempdir() + "/" + text_fingerprint + ".mp3"

        self.generate_tts_file(speech, filename)
        self.play_sound_file(filename)

        if not use_cache:
            os.remove(filename)

    def generate_tts_file(self, speech, filename):

        url = "https://api.elevenlabs.io/v1/text-to-speech/xt0y2vcn6RmawK03ZEfJ?output_format=mp3_22050_32"

        headers = {
            "Accept": "audio/mpeg",
            "Content-Type": "application/json",
            "xi-api-key": ELEVEN_LABS_API_KEY
        }

        data = copy.copy(SPEECH_SETTINGS)
        data["text"] = speech

        response = requests.post(url, json=data, headers=headers)

        if response.status_code >= 300:
            response.raise_for_status()

        with open(filename, "wb") as f:
            for chunk in response.iter_content(chunk_size=TTS_CHUNK_SIZE):
                if chunk:
                    f.write(chunk)

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
        subprocess.run(['bluetoothctl', '--timeout=15', 'scan', 'on'])
        subprocess.run(['bluetoothctl', 'pair', audio_device_addr])
        subprocess.run(['bluetoothctl', 'trust', audio_device_addr])
        subprocess.run(['bluetoothctl', 'connect', audio_device_addr])


def main(args=None):
    rclpy.init(args=args)

    node = SoundEngine()

    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
