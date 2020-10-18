#ifndef SCAN_STRUCT_h
#define SCAN_STRUCT_h


#define BLOCKS_PER_PACKAGE      12
#define HEADER_BYTES            42
#define FLAG_BYTES              2
#define AZIMUTH_BYTES           2
#define CHANNELS                16
#define CHANNEL_SETS_PER_BLOCK  2
#define DISTANCE_BYTES          2
#define REFLECTIVITY_BYTES      1
#define TIMESTAMP_BYTES         4
#define FACTORY_BYTES           2
#define BLOCK_BYTES             (FLAG_BYTES + AZIMUTH_BYTES + CHANNEL_SETS_PER_BLOCK*CHANNELS*(DISTANCE_BYTES + REFLECTIVITY_BYTES))
#define PACKAGE_BYTES           (BLOCKS_PER_PACKAGE * BLOCK_BYTES  + TIMESTAMP_BYTES + FACTORY_BYTES)
#define PACKAGE_FACTORY_INDEX   (PACKAGE_BYTES - FACTORY_BYTES)
#define PACKAGE_TIMESTAMP_INDEX (PACKAGE_FACTORY_INDEX - TIMESTAMP_BYTES)

/*
Loading from data options:
    unsigned long test1;
    memcpy(&test1, &pkt.packets[0].data[PACKAGE_TIMESTAMP_INDEX], 4);

    unsigned long * test2 = (unsigned long *) (&pkt.packets[0].data[PACKAGE_TIMESTAMP_INDEX]);

    unsigned long test3 = *((unsigned long *) &pkt.packets[0].data[PACKAGE_TIMESTAMP_INDEX]);
*/

struct __attribute__((__packed__)) VlpData{
  uint16_t distance;
  uint8_t reflectivity;
};
struct __attribute__((__packed__)) VlpDataSet{
  VlpData channels[CHANNELS];
};
struct __attribute__((__packed__)) VlpBlock {
  uint16_t flag;
  uint16_t azimuth;
  VlpDataSet dataset[2];
};
struct __attribute__((__packed__)) Header {
  uint32_t header_01; // 4
  uint32_t header_02; // 8
  uint32_t header_03; // 12
  uint32_t header_04; // 16
  uint32_t header_05; // 20
  uint32_t header_06; // 24
  uint32_t header_07; // 28
  uint32_t header_08; // 32
  uint32_t header_09; // 36
  uint32_t header_10; // 40
  uint16_t header_11; // 42
};
struct __attribute__((__packed__)) Package {
  //Header header; //Removed by velodyne driver
  VlpBlock blocks[12];
  uint32_t time_stamp;
  uint16_t factory;
};

#endif