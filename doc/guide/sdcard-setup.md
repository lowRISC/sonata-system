# Setting up SD card image for 'sdcard_tests.hh'

To set up a FAT32-formatted SD card image 'sd.img' on a Linux system, the following
may be useful:

https://github.com/procount/fat32images offers various ready-to-download volume images.
The 1GiB example (nnobs1gb.img.zip) should suffice; rename it to 'sd.img'

Find the offset of the FAT32 partition:

> fdisk -u sd.img
```
Disk sd.img: 1 GiB, 1073741824 bytes, 2097152 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x6c4676d7

Device        Boot Start     End Sectors  Size Id Type
noobs1gb.img1       8192 2097151 2088960 1020M  c W95 FAT32 (LBA)
```

Note the 'Start' value (8192) and multiply it by the sector size (512 bytes),
e.g. 4194304 in the command below:

Create a mount point within the filing system for the FAT32 partition.
> mkdir mnt_point

Make the FAT32 partition available.
> sudo mount -o loop,offset=4194304 mnt_point sd.img

Copy the lorem ipsum text (or other files) into the root directory of the FAT32 partition.
> sudo cp <lorem file> mnt_point/lorem.ips

Unmount the FAT32 partition.
> sudo umount mnt_point

