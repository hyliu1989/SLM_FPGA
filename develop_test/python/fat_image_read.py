def littleEndianParse(bytes_arr):
    ret = 0
    for i in range(len(bytes_arr)):
        ret = ret + (bytes_arr[i] << i*8)
    return ret

def displayInfo(in_file):
    assert in_file.mode == 'rb'

    # go to the beginning of the FAT32 image
    in_file.seek(0)
    boot_section = in_file.read(64)
    in_file.seek(0)

    # FAT32 check
    assert boot_section[0x11] == 0
    assert boot_section[0x12] == 0
    assert boot_section[0x16] == 0
    assert boot_section[0x17] == 0

    # number of bytes per sector
    n_bytes_per_sector = littleEndianParse(boot_section[0xb:0xb+2])

    # number of sectors per cluster
    n_sectors_per_cluster = boot_section[0xd]

    # number of reserved sectors in front of File Allocation Tables (FATs)
    n_reserved_sectors = littleEndianParse(boot_section[0xe:0xe+2])

    # number of FATs
    n_FATs = boot_section[0x10]

    # number of sectors per FAT
    n_sectors_per_FAT = littleEndianParse(boot_section[0x24:0x24+4])

    # the cluster id of root directory
    id_of_root_cluster = littleEndianParse(boot_section[0x2c:0x2c+4])

    print('number of bytes per sector:              %d' % n_bytes_per_sector)
    print('number of sectors per cluster:           %d' % n_sectors_per_cluster)
    print('number of reserved sectors before FATs:  %d' % n_reserved_sectors)
    print('number of FATs:                          %d' % n_FATs)
    print('number of sectors per FAT:               %d' % n_sectors_per_FAT)
    print('id number of root cluster:               %d' % id_of_root_cluster)

    print('---------- main output ------------')
    # byte location of FAT
    idx_file_alloc_table = n_reserved_sectors * n_bytes_per_sector
    print('FAT index:                             0x%X' % idx_file_alloc_table)

    # byte location of root directory
    idx_root_directory = (n_reserved_sectors + n_FATs*n_sectors_per_FAT) * n_bytes_per_sector
    print('root directory index:                  0x%X' % idx_root_directory)


def fileEntryParsing(bytes_arr, file_name='masks.npy'):
    """Parse the entry in root directory. 
    """
    assert len(bytes_arr) >= 32

    if (bytes_arr[11] & 0xF) == 0xF:
        return 'Entry is of long-name format'

    # convert input file name to bytes
    assert len(file_name) < 12
    name_root = file_name.split('.')[0].upper().encode()
    name_ext = file_name.split('.')[-1].upper().encode()

    # check the name part of the bytes array
    byte_arr_name_part = bytes_arr[:11]
    if byte_arr_name_part[0] == 0xE5:  # file is deleted
        return 'file is deleted'
    is_right_file = (byte_arr_name_part[:len(name_root)] == name_root)
    is_right_file = (byte_arr_name_part[-len(name_ext):] == name_ext) and is_right_file
    space_part = byte_arr_name_part[len(name_root):-len(name_ext)]
    is_right_file = (space_part == b' '*len(space_part)) and is_right_file
    if not is_right_file:
        return '(%s) Not the right file' % byte_arr_name_part.decode('ascii')

    # get the location of the file, in unit of clusters
    temp = bytes_arr[0x1A:0x1A+2] + bytes_arr[0x14:0x14+2]  # low digits + high digit, because of little endianness so high is behind low
    first_cluster = littleEndianParse(temp)
    print('(%s) The first cluster of the file is 0x%X' % (byte_arr_name_part.decode('ascii'),first_cluster))

    return True


def genTestArrayOfMasks():
    import numpy as np
    np.random.seed((45,2,343))
    return np.random.randint(256, size=(64,1024,1024)).astype(np.uint8)



if __name__ == '__main__':
    in_file = open('../../../microSD.img','rb')  # load an image of FAT32 system
    print('opened file is bound with variable "in_file".')

    displayInfo(in_file)
