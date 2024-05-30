import rclpy
from rclpy.node import Node
import subprocess, os, time
from std_msgs.msg import String
import re
import hashlib
import json
from elevenlabs.client import ElevenLabs
from elevenlabs import Voice, stream, save

BT_ADAPTER_ADDR = os.environ.get("BT_ADAPTER_ADDR") or "8C:88:0B:4A:27:A8"
BT_SPEAKER_ADDR = os.environ.get("BT_SPEAKER_ADDR") or "12:11:CE:D4:E8:3A"  # SONY: "6C:47:60:9B:98:F5"
ELEVEN_LABS_API_KEY = os.environ.get("ELEVEN_LABS_API_KEY")
SPEECH_VOICE_ID = "xt0y2vcn6RmawK03ZEfJ"
SPEECH_SETTINGS = {
    "model_id": "eleven_multilingual_v2",
    "voice_settings": {
        "stability": 0.5,
        "similarity_boost": 0.5,
        "style": 0.1,
        "use_speaker_boost": True
    }
}

el_voice = Voice(
    voice_id=SPEECH_VOICE_ID,
    settings=SPEECH_SETTINGS["voice_settings"]
)

el_client = ElevenLabs(
    api_key=ELEVEN_LABS_API_KEY
)


class SoundEngine(Node):

    def __init__(self):
        super().__init__('sound_engine')

        self.bt_adapter_addr = BT_ADAPTER_ADDR
        self.bt_speaker_addr = BT_SPEAKER_ADDR
        self.mp3_player = None
        self.bt_sound_system_proc = None
        self.sound_system_ready = False

        self.speech_settings_hash = hashlib.md5(json.dumps(SPEECH_SETTINGS, sort_keys=True).encode()).hexdigest()

        self.sound_cache_dir = os.environ.get("LAWNNY5_CACHE") + '/sounds'
        self.premade_sound_dir = os.environ.get("LAWNNY5_ASSETS") + "/sounds"

        self.create_subscription(
            String,
            'sound/play/by_name',
            self.handle_play_by_name,
            1)

        self.create_subscription(
            String,
            'sound/speak/text',
            self.handle_speak_text,
            1)

        self.sound_status_publisher = self.create_publisher(String, "sound/status", 1)

        self.startup()

    def startup(self):
        self.connect_bluetooth_speaker(self.bt_adapter_addr, self.bt_speaker_addr)

    def shutdown(self):
        self.disconnect_bluetooth_speaker(self.bt_speaker_addr)

    def handle_play_by_name(self, msg):
        self.play_sound_by_name(msg.data)

    def handle_speak_text(self, msg):
        self.speak_text(msg.data)

    def play_sound_by_name(self, name):
        self.play_sound_file(self.premade_sound_dir + "/" + name + '.mp3')

    def speak_text(self, speech):

        if not self.sound_system_ready:
            self.get_logger().error("Sound system not ready to play speech.")
            return

        status_msg = String()
        status_msg.data = "generating_speech"
        self.sound_status_publisher.publish(status_msg)

        # self.generate_tts_file(speech)
        audio = el_client.generate(
            text=speech,
            voice=el_voice,
            model=SPEECH_SETTINGS["model_id"],
            stream=True
        )

        stream(audio)

        status_msg.data = "speech_generated"
        self.sound_status_publisher.publish(status_msg)

    # def generate_tts_file(self, speech):
    #     audio = el_client.generate(
    #         text=speech,
    #         voice=el_voice,
    #         model=SPEECH_SETTINGS["model_id"],
    #         stream=True
    #     )
    #
    #     stream(audio)

    def play_sound_file(self, file):
        self.get_logger().info("Playing sound: %s" % file)
        status_msg = String()
        status_msg.data = "playing"
        self.sound_status_publisher.publish(status_msg)
        # subprocess.run(['mpg123', file], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["mpv", "--no-cache", "--no-terminal", file], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        status_msg.data = "stopped"
        self.sound_status_publisher.publish(status_msg)

    def disconnect_bluetooth_speaker(self, audio_device_addr):
        self.sound_system_ready = False

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

        status_msg = String()
        status_msg.data = "audio_system_ready"
        self.sound_status_publisher.publish(status_msg)
        self.sound_system_ready = True


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
