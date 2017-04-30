#ifndef __SEGMENT_H_
#define __SEGMENT_H_

#include <stdint.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "huffman.h"

typedef enum {
  SOI,
  APP0,
  DQT,
  SOF0,
  DHT,
  DRI,
  SOS,
  EOI,
  UNKNOWN
} MarkerType;

struct Segment {
  MarkerType type;
  std::vector<uint8_t> buf;
  const int size() const { return buf.size(); }
};

struct JPEGData {
  struct Comp {
    uint8_t c,h,v,tq,td,ta;
    int width,height;
  };
  uint16_t height, width;
  uint8_t maxH, maxV;
  int16_t qt[4][64];
  std::vector< std::vector<uint8_t> > YCbCr[4];
  std::vector<Comp> comp;
  Huffman huf[2][2];
  std::ifstream ip;

  ~JPEGData ();

  int nComp();
  void info();
};

MarkerType get_marker_type(uint16_t marker);
Segment read_segment(std::ifstream &jpg);

bool parse_SOF0(const Segment &seg, JPEGData &jpg);
bool parse_DQT(const Segment &seg, JPEGData &jpg);
bool parse_DHT(const Segment &seg, JPEGData &jpg);
bool parse_SOS(const Segment &seg, JPEGData &jpg);

#endif
