#include <cstdio>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <unistd.h>
#include <assert.h>
#include "jpeg_decoder.h"

using namespace std;

JPEGData jpg;

int main(int argc, char** argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <file>\n", argv[0]);
    return 1;
  }
  jpg.ip.open(argv[1], ios::in | ifstream::binary);
  if (!jpg.ip) {
    perror(argv[1]);
    return 1;
  }
  Segment seg;
  seg = read_segment(jpg.ip);
  assert(seg.type == SOI);
  
  while (true) {
    seg = read_segment(jpg.ip);
    if (seg.type == EOI) break;

    switch (seg.type) {
			case SOF0:
        if (!parse_SOF0(seg, jpg)) {
          cerr<<"SOF0 ERROR"<<endl;
          return 1;
        }
        break;
			case APP0:
        // pass
        break;
			case DQT:
        if (!parse_DQT(seg, jpg)) {
          cerr<<"DQT ERROR"<<endl;
          return 1;
        }
        break;
			case DHT:
        if (!parse_DHT(seg, jpg)) {
          cerr<<"DHT ERROR"<<endl;
          return 1;
        }
        break;
			case SOS:
        parse_SOS(seg, jpg);
        break;
			case DRI:
        //parse_DRI(seg);
        break;
      default:
        ;
    }
  }
  cout<<"Save the result to output.bmp"<<endl;

  return 0;
}

