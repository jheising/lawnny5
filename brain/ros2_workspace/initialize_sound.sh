#!/bin/bash

set +H
echo -e "pcm.btspeak {
    type plug
    slave.pcm {
        type bluealsa
        device \"$AUDIO_DEVICE_ADDR\"
        profile \"a2dp\"
    }
}

pcm.btmic {
	type plug
	slave.pcm {
		type bluealsa
		device \"$AUDIO_DEVICE_ADDR\"
		profile \"sco\"
	}
}

pcm.!default {
	type asym
	capture.pcm \"btmic\"
	playback.pcm \"btspeak\"
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