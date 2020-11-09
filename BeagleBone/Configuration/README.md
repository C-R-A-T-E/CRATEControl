# BeagleBone AI setup notes

Comes from TI with account `debian:tempwd`, hostname beaglebone.

## Set debian password

    ssh debian@beaglebone
    passwd

## Update installed packages

    sudo apt update && sudo apt upgrade

## Install required packages

Lightweight Communicaions & Marshling (LCM) [link](https://lcm-proj.github.io/)
GNU Project Debugger (GDB) [link](https://www.gnu.org/software/gdb/)

    sudo apt install liblcm-dev liblcm-bin gdb

## Upgrade scripts

Must use `debian` account. Copied from [here](https://beagleboard.org/upgrade)

    cd /opt/scripts
    git pull
    sudo tools/update_kernel.sh

## Set hostname

The preferred hostname is `crate-<function>`

    sudo vim /etc/hostname

## User Account for developers

    sudo adduser <username>
    GRPS=$(groups | cut -d ' ' -f 2- | tr ' ' ',')
    sudo usermod -a -G $GRPS <username>

## Account for CRATE operation

    sudo adduser crate
    GRPS=$(groups | cut -d ' ' -f 2- | tr ' ' ',')
    sudo usermod -a -G $GRPS <username>

## disable USB networking (don't need it, adds noise to the logs)

In `/etc/default/bb-boot`, add the line (or uncomment)

    USB_NETWORK_DISABLED=yes

In `/etc/network/interfaces`, comment out the whole `iface usb0` section.

In `/opt/scripts/boot/bbai.sh`, comment out the two blocks starting around line
368 that run `autocnifgure_usb[01].sh`.

## Ensure a multicast route always exists.
If LCM won't start at boot, this is probably the solution.

Add these two `post-up` lines to `/etc/network/interfaces`. The rest should
already be there:

    auto lo
    iface lo inet loopback
      post-up ifconfig lo multicast || true
      post-up route add -net 224.0.0.0 netmask 240.0.0.0 dev lo || true


## Reboot

    sudo shutdown --reboot now
