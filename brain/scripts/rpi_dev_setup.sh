#!/bin/sh

#SSH_KEY="$(>&2 echo -n "Enter your SSH public key: "; read u; echo $u)"
#mkdir -p ~/.ssh
#echo "$SSH_KEY" >> ~/.ssh/authorized_keys

MOUNT_USERNAME="$(>&2 echo -n "Enter share username: "; read u; echo $u)"
SSH_CLIENT_IP="$(echo $SSH_CLIENT | awk '{ print $1}')"

mkdir -p /home/lawnny/src

sudo mount -t cifs //"$SSH_CLIENT_IP"/development /home/lawnny/src -o username="$MOUNT_USERNAME",sec=ntlmssp,nounix