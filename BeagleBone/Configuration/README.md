# BeagleBone AI setup notes

Comes from TI with account `debian:temppwd`, hostname beaglebone.

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

### vim

    sudo vim /etc/hostname
    
### emacs
    
    sudo emacs -nw /etc/hostname
    
    (launches the non-windowed (terminal-shell-based) version of emacs, if installed).
    See instructions below for running emacs in windowed mode
    
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
368 that run `autoconfigure_usb[01].sh`.

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
    
## Running emacs in windowed mode on a connected host machine
    
    After launching emacs in a window on the connected host machine, type
    
    C-x C-f
    
    which is the usual command for opening a file. This will put you in the minibuffer.
    Backspace over the default text in the minibuffer, if any, then type
    
    /ssh:<username>@<hostname>|sudo:<hostname>:/<filename on the BBAI you want to edit>
    
    where <username> is the sudo-enabled user on the BBAI and <hostname> is the BBAI's
    hostname.
    
    for example, for user 'nts' on the BBAI 'CRATE'
    
    /ssh:nts@CRATE|sudo:CRATE:/home/nts/test.txt
    
    After a brief delay you'll be asked for the su password on the BBAI. After that hit 
    return when asked about saving credentials and you should see the file open up in
    a new emacs buffer. From that point editing should work as normal except for slight
    delays and extra messages in the minibuffer when saving, etc.
