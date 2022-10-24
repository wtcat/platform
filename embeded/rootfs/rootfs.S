/*
 * Copyright 2022 wtcat
 */
    .section ".rodata.rootfs", "a"
    .align 8
    .global __rootfs_image
    .global __rootfs_image_end
    .global __rootfs_size

__rootfs_image:
    .incbin "rootfs.tar"
__rootfs_image_end:
    .set __rootfs_size, __rootfs_image_end - __rootfs_image
    .align 8
    .end
