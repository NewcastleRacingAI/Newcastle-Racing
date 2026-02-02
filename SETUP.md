# Setting up a development enviroment
## Sections
1. [Introduction](#Introduction)
2. [Obtaining a supported environment](#Obtaining)
    1. [Windows](#Windows)
    2. [Linux](#Linux)
    3. [MacOS](#MacOS)
    4. [FreeBSD](#FreeBSD)
3. [Setting up ros2](#Setting)
    1. [Windows-Setup](#Windows-Setup)
    2. [Linux-Setup](#Linux-Setup)

## Introduction

The following is the preliminary reccomendation for setting up a development
environment for [ros2 humble](https://docs.ros.org/en/humble/).
feedback is appreciated

## Obtaining a supported environment

The following section explains the reccomended way to obtain a supported environment accross multiple different host systems

### Windows

Windows is a [tier 1 supported](https://reps.openrobotics.org/rep-2000/#humble-hawksbill-may-2022---may-2027)
operating system you can skip to [Setting up ros2]   

Alternatively you could install a [supported linux distribution](https://reps.openrobotics.org/rep-2000/#humble-hawksbill-may-2022---may-2027) in WSL and continue via the [Linux-Setup](#Linux-Setup).

### Linux

Users who already run a distribution that is [tier 1 supported](https://reps.openrobotics.org/rep-2000/#humble-hawksbill-may-2022---may-2027) by ros2 humble may skip this section.   

Users who do not use a tier 1 supported distribution **OR** who do not wish to
pollute their install with ros2 development, may continue following this section
to set up an ubuntu chroot environment which they can develop ros2 nodes inside.

#### What is a chroot?
The purpose of a chroot is to run programs under a seperate root directory to the host system while still sharing the underlying kernel. In our case this has the advantage of us to be able to simulate an ubuntu system with essentialy none of the overhead that comes with solution like virtual machines. Also unlike a container or virtual machine a chroots filesystem is fully transparent to the host system. While a chroot does seperate the environments filesystem from the host, it does not offer the same strong security benifits of virtual machines, for example a malicious program with root access within a chroot would be able to take controll of the host system.


#### Setting up the ubuntu system
first switch to the root user
```bash
su root
```
if your distribution locks the root user you may instead run commands through `sudo` or systemd's `run0`   

next change dir to wherever you want the files for your chroot to reside if you have no preference /opt will do
```bash
cd /opt
```
your system should provide the debootstrap utility through your package manager install it and run this command in the directory you wish for your chroot files to be stored
> '--arch=' is only required if your system didnt pull in dpkg as a dependency of debootstrap but it doesnt hurt to include it anyway
>> use '--arch=amd64' if your cpu is made by amd OR intel
>> use '--arch=aarch64' if your cpu is an arm processer (prob isnt unless apple m-series)
```bash
debootstrap --arch=amd64 --variant=buildd jammy ./racing-ubuntu http://archive.ubuntu.com/ubuntu/
```
after that command finishes run the following command to create a script in the current directory to enter the chroot
```bash
echo -e '#!/usr/bin/bash' "\n\
\n\
if ! ((\$(echo \$UID) == 0)); then\n\
        >&2 echo \"error user must be root\"\n\
        exit 1\n\
fi\n\
\n\
USER=\"root\"\n\
CHROOT_PATH=\"$(cd ./racing-ubuntu && pwd && cd ..)\"\n\
mount --onlyonce --rbind /dev \$CHROOT_PATH/dev\n\
mount --onlyonce --make-rslave \$CHROOT_PATH/dev\n\
mount --onlyonce -t proc /proc \$CHROOT_PATH/proc\n\
mount --onlyonce --rbind /sys \$CHROOT_PATH/sys\n\
mount --onlyonce --make-rslave \$CHROOT_PATH/sys\n\
mount --onlyonce --rbind /tmp \$CHROOT_PATH/tmp\n\
mount --onlyonce --bind /run \$CHROOT_PATH/run\n\
\n\
# we use env -i here to remove existing environment variables from the host system\n\
# incorrect env variables are far worse than no env variables\n\
# use su to re-login to your user account within the chroot if you need env vars\n\
if [[ \$USER =~ ^\"root\" ]] ; then\n\
        env -i HOME=\"/\$USER\" chroot --userspec=\"\$USER:\$USER\" \$CHROOT_PATH /usr/bin/bash --login\n\
else\n\
        env -i HOME=\"/home/\$USER\" chroot --userspec=\"\$USER:\$USER\" \$CHROOT_PATH /usr/bin/bash --login\n\
fi" > chroot-racing-ubuntu && chmod +x ./chroot-racing-ubuntu
```

execute the script to enter the chroot
> If you enter the chroot more than once per session you will get an error `filesystem already mounted.` this is normal and can be safely ignored
```bash
./chroot-racing-ubuntu
```

change the chroot prompt to avoid confusion
```bash
echo "export PS1=\"(chroot) \$PS1\"" >> /root/.bashrc && source /root/.bashrc
```

set the root password
```bash
passwd
```

Initialise our local package database
```bash
apt update
```

You can now install any personal packages that you may want to use e.g.
```bash
apt install nano sudo
```
will install `nano` and `sudo`   

we can now add a user to the chroot environment
> If you choose to rename you user replace **larry** with the desired name of your user anywhere you see **larry** in this tutorial you should replace it with your user name before executing the command
```bash
groupadd wheel
```
```bash
useradd -m -G users,wheel -s /usr/bin/bash larry
```

set the user password
```bash
passwd larry
```

and make the same addition to the prompt that we did for the root user
```bash
echo "export PS1=\"(chroot) \$PS1\"" >> /home/larry/.bashrc
```

we can now exit the chroot using
```bash
exit
```
or
```bash
logout
```
> for non login sessions such as using `su` without `-l` exit must be used instead

finaly we can edit the `chroot-racing-ubuntu` script to log us into the user we have just created rather than the root user. To do this change the variable `user` from `root` to the name that you chose for you user e.g. `USER="larry"`

#### Troubbleshooting
you may find the lack of enviroment variables an issue for certain tasks to fix this you should spawn a nested shell within you chroot by using
```bash
su larry
```

### MacOS

***TODO*** (linux container?)

### FreeBSD

**disclaimer** - this is untested   

It should be possible to follow the [Setting up linux](#Setting Up Linux)
instructions after enabling [Linux binary compatability](https://docs.freebsd.org/en/books/handbook/linuxemu/)
and installing an [ubuntu base system with debootstrap](https://docs.freebsd.org/en/books/handbook/linuxemu/#linuxemu-debootstrap)
similar to the [Linux](#Linux) instructions. Howver there may be issues that
arise from the lack of systemd.

## Setting up ros2

### Windows-Setup

> For Users wishing to use WSL please install a [supported distrobution](https://reps.openrobotics.org/rep-2000/#humble-hawksbill-may-2022---may-2027) (Ubuntu jammy) and continue by following the [Linux-Setup](#Linux-Setup).   

**TODO** (gabe)

### Linux-Setup

The linux install instructions provided by ros2 are generaly sufficently detailed so I wont repeat them here   
[ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)   

[RHEL based distrobutions](https://docs.ros.org/en/humble/Installation/RHEL-Install-RPMs.html)   

However it is worth noting that users using an ubuntu chroot may not be able to use graphical applications from within the chroot, due to not passing in environment variables to the chroot from the host. See [this article](https://wiki.gentoo.org/wiki/Chroot#Wayland) for more information about running graphical applications in a chroot. You are still able to use Graphical applicaions on the host system to edit files within the chroot as you would any other file on your system.   

The ros2 doccumentaion suggests [confuiguring the evironment](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#environment-setup) using `source /opt/ros/humble/setup.bash` however this will only apply to the current shell i.e. if you close the current shell or spawn a new one this will no longer apply. To make this command apply to all future shells run the following command
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
