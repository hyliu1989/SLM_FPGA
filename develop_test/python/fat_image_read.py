def little_endian_parse(bytes_arr):
    ret = 0
    for i in range(len(bytes_arr)):
        ret = ret + (bytes_arr[i] << i*8)
    return ret

def display_info(in_file):
    assert in_file.mode == 'rb'

    # go to the beginning of the FAT32 image
    in_file.seek(0)

    boot_section = in_file.read(64)

    # FAT32 check
    assert boot_section[0x11] == 0
    assert boot_section[0x12] == 0
    assert boot_section[0x16] == 0
    assert boot_section[0x17] == 0

    # number of bytes per sector
    n_bytes_per_sector = little_endian_parse(boot_section[0xb:0xb+2])

    # number of sectors per cluster
    n_sectors_per_cluster = boot_section[0xd]

    # number of reserved sectors in front of File Allocation Tables (FATs)
    n_reserved_sectors = little_endian_parse(boot_section[0xe:0xe+2])

    # number of FATs
    n_FATs = boot_section[0x10]

    # number of sectors per FAT
    n_sectors_per_FAT = little_endian_parse(boot_section[0x24:0x24+4])

    # the cluster id of root directory
    id_of_root_cluster = little_endian_parse(boot_section[0x2c:0x2c+4])

    print('number of bytes per sector:              %d' % n_bytes_per_sector)
    print('number of sectors per cluster:           %d' % n_sectors_per_cluster)
    print('number of reserved sectors before FATs:  %d' % n_reserved_sectors)
    print('number of FATs:                          %d' % n_FATs)
    print('number of sectors per FAT:               %d' % n_sectors_per_FAT)
    print('id number of root cluster:               %d' % id_of_root_cluster)

    # byte location of root directory
    idx_root_directory = (n_reserved_sectors + n_FATs*n_sectors_per_FAT) * n_bytes_per_sector
    print('root directory index:                    %d' % idx_root_directory)

