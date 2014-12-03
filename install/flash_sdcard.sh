#! /bin/bash

# Forcing root session
sudo pwd > /dev/null
echo "> Welcome to SD card flashing utility."
echo -n "> Insert a SD card to format, then press [ENTER]..."
read

# Input of the SD card drive name
echo -n "> Please enter the name of the SD card device (default: sdb): "
read drive
if [ -n "$drive" ]; then
    echo "> Drive /dev/"${drive}" will be formatted."
else
    echo "> Drive /dev/sdb will be formatted."
    drive="sdb"
fi
echo "> Warning, all data will be erased on "${drive} "!"
echo "> Please verify that the disk below is the one you want to flash."
/sbin/fdisk -l |grep /dev/${drive}
echo -n "> Press [ENTER] to continue or [CTRL-C] to abort..."
read

# Partitioning
echo -n "> Repartitioning SD card..."
sudo umount /dev/${drive}1 &> /dev/null
sudo umount /dev/${drive}2 &> /dev/null
sudo fdisk /dev/${drive} < fdisk.cmd &>> sdcard.err
echo "done."

# Formatting
echo -n "> Formatting DOS partition..."
sudo mkfs.msdos -n LMS2012 /dev/${drive}1 &>> sdcard.err
echo "done."
echo -n "> Formatting ext3 partition..."
sudo mkfs.ext3 -L LMS2012_EXT /dev/${drive}2 &>> sdcard.err
echo "done."
if [ -e /dev/${drive}1 ]
then
	if [ -e /dev/${drive}2 ]
  then
		echo "> SD card was successfully formatted."
  else
		echo "> Formatting of ext3 partition went into some troubles. Check for errors below."
		cat sdcard.err
		exit -1
  fi
else
	echo "> Formatting of DOS partition went into some troubles. Check for errors below."
	cat sdcard.err
	exit -1
fi

# Mounting SD card partitions
sudo umount /dev/${drive}1 &> /dev/null
sudo umount /dev/${drive}2 &> /dev/null

# Udpating kernel filesystem
if sudo mount /dev/${drive}1 ./LMS2012;
then
	echo -n "> Copying the kernel..."
	sudo cp uImage ./LMS2012/uImage
  sync
  echo "done."
else
	echo "> Unable to mount kernel filesystem. Aborting."
	exit -2
fi

# Updating the root filesystem
if sudo mount /dev/${drive}2 ./LMS2012_EXT;
then
	echo -n "> Copying root filesystem to SD card..."
	sudo tar zfx devices.tgz  -C ./LMS2012_EXT
	sudo cp -a fs/* ./LMS2012_EXT
	sudo chown -R root.root ./LMS2012_EXT/*
	sync
	echo "done."
	echo -n "> Copying ev314 software to SD card..."
	sudo cp -r ../ev314/Linux_AM1808/sys ./LMS2012_EXT/home/root/ev314
	sync
	echo "done."
else
	echo "> Unable to mount the root filesystem of the SD card. Aborting."
	exit -3
fi

# Unmounting SD card
sudo umount /dev/${drive}1 &> /dev/null
sudo umount /dev/${drive}2 &> /dev/null
echo "> You can now eject the SD card and test it in a EV3."

