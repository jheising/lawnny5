#!/bin/bash

# Replace with your bluetooth device dongle ID
#export BT_DEVICE="8C:88:0B:4A:27:A8"
#export SPEAKER_DEVICE="12:11:CE:D4:E8:3A"

set +H
echo -e "pcm.btreceiver {
    type plug
    slave.pcm {
        type bluealsa
        device \"$AUDIO_DEVICE_ADDR\"
        profile \"a2dp\"
    }
    hint {
        show on
        description \"Bluetooth Receiver\"
    }
}

pcm.!default {
 type plug
    slave.pcm \"btreceiver\"

}" > /etc/asound.conf
set -H

# Restart DBUS
sudo service dbus restart

# Start bluealsa
sudo bluealsa &

sleep 0.25

# Start our bluetooth daemon
sudo bluetoothd &

wait