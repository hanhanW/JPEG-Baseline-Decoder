#include "segment.h"
#include "utils.h"
#include "bitmap/bitmap_image.hpp"
#include <cstdlib>
#include <cmath>
#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>

using namespace std;

MarkerType get_marker_type(uint16_t marker) {
  switch (marker) {
    case 0xFFD8: return SOI;
    case 0xFFE0: return APP0;
    case 0xFFDB: return DQT;
    case 0xFFC0: return SOF0;
    case 0xFFC4: return DHT;
    case 0xFFDD: return DRI;
    case 0xFFDA: return SOS;
    case 0xFFD9: return EOI;
    default: return UNKNOWN;
  }
}

Segment read_segment(std::ifstream &jpg) {
  Segment res;
  uint16_t val, marker, length;

  jpg.read(reinterpret_cast<char*>(&val), 2);
  marker = econv16(val);
  res.type = get_marker_type(marker);
  if (res.type == SOI or res.type == EOI) return res;

  jpg.read(reinterpret_cast<char*>(&val), 2);
  length = econv16(val);

  if (length) {
    res.buf.resize(length-2);
    jpg.read(reinterpret_cast<char*>(&res.buf[0]), length-2);
  }
#ifdef VERBOSE
  fprintf(stderr, "read segment, marker = %x, len = %d\n", marker, length);
#endif

  return res;
}


JPEGData::~JPEGData () {
  if (ip.is_open()) {
    ip.close();
  }
}

int JPEGData::nComp() {
  return comp.size();
}

bool parse_SOF0(const Segment &seg, JPEGData &jpg) {
  if (seg.buf[0] != 8) {
    cerr<<"SOF0 P != 8"<<endl;
    return false;
  }
  jpg.height = ((int16_t)seg.buf[1] << 8) + seg.buf[2];
  jpg.width  = ((int16_t)seg.buf[3] << 8) + seg.buf[4];
  jpg.comp.resize(seg.buf[5]);

#ifdef VERBOSE
  cerr<<"Resolution: "<<jpg.width<<"x"<<jpg.height<<endl;
#endif

  if (6 + (int)seg.buf[5] * 3 > seg.size()) {
    cerr<<"SOF0 parsing components error"<<endl;
    return false;
  }

  jpg.maxH = jpg.maxV = 0;

  for (int i=0, pos=6; i<jpg.nComp(); i++) {
    jpg.comp[i].c = seg.buf[pos++];
    jpg.comp[i].h = (seg.buf[pos] >> 4) & 0xF;
    jpg.comp[i].v = (seg.buf[pos++]) & 0xF;
    jpg.comp[i].tq = seg.buf[pos++];
#ifdef VERBOSE
    cerr<<"Component "<<i<<": C = "<<(int)jpg.comp[i].c<<", H = "<<(int)jpg.comp[i].h<<", V = "
      <<(int)jpg.comp[i].v<<", Tq = "<<(int)jpg.comp[i].tq<<endl;
#endif
    jpg.maxH = max(jpg.maxH, jpg.comp[i].h);
    jpg.maxV = max(jpg.maxV, jpg.comp[i].v);
  }

  for (int i=0; i<jpg.nComp(); i++) {
    jpg.comp[i].width = (int)jpg.width * jpg.comp[i].h / jpg.maxH;
    jpg.comp[i].height = (int)jpg.height * jpg.comp[i].v / jpg.maxV;
#ifdef VERBOSE
    cerr<<"Component "<<i<<": (x, y) = ("<<jpg.comp[i].width<<", "<<jpg.comp[i].height<<")"<<endl;
#endif

    jpg.YCbCr[i].resize(jpg.comp[i].height);
    for (int j=0; j<jpg.comp[i].height; j++) {
      jpg.YCbCr[i][j].resize(jpg.comp[i].width);
    }
  }

  return true;
}

bool parse_DQT(const Segment &seg, JPEGData &jpg) {
  for (int offset=0, pos; offset<seg.size(); offset=pos) {
    pos = offset;
    int Pq = (seg.buf[pos] >> 4) & 0xF;
    assert(Pq == 0);
    int Tq = (seg.buf[pos++]) & 0xF;

    if (pos + 64 > seg.size()) {
      cerr<<"DQT size ERROR"<<endl;
      return false;
    }
    for (int i=0; i<64; i++) {
      jpg.qt[Tq][i] = seg.buf[pos++];
    }

    int16_t mat[8][8];
    zigzag(jpg.qt[Tq], mat); 
#ifdef VERBOSE
    cerr<<"Tq = "<<Tq<<endl;
    cerr<<"Quantization table:\n";
    for (int i=0; i<8; i++)
      for (int j=0; j<8; j++)
        cerr<<(int)mat[i][j]<<" \n"[j==7];
#endif
  }

  return true;
}

void print_code(int code, int len)
{
  for (int i = len - 1; i >= 0; i--) fputc('0' + ((code >> i) & 1), stderr);
  for (int i = 15; i >= len; i--) fputc(' ', stderr);
}

bool parse_DHT(const Segment &seg, JPEGData &jpg) {
  int lenCnt[17];
  for (int offset=0, pos; offset<seg.size(); offset=pos) {
    pos = offset;
    int Tc = (seg.buf[pos] >> 4) & 0xF;
    int Th = (seg.buf[pos++]) & 0xF;
    for (int i=1; i<=16; i++) lenCnt[i] = seg.buf[pos++];

#ifdef VERBOSE
    cerr<<"Tc = "<<Tc<<", Th = "<<Th<<endl;
    cerr<<"# of length 1-16:";
    for (int i=1; i<=16; i++) cerr<<" "<<lenCnt[i];
    cerr<<endl;
#endif

    for (int len=1, code=0; len<=16; len++) {
      for (int i=0; i<lenCnt[len]; i++) {
        if (pos >= seg.size()) {
          cerr<<"DHT parsing error"<<endl;
          return false;
        }
        jpg.huf[Tc][Th].insert(code, len, seg.buf[pos++]);
#ifdef VERBOSE
        print_code(code, len);
        fprintf(stderr, ": 0x%02x\n", seg.buf[pos-1]);
#endif
        code++;
      }
      code <<= 1;
    }
  }
  return true;
}

uint8_t decode(Huffman &huf, uint8_t &byte, int &blen, JPEGData &jpg) {
  int id = huf.root;
  while (huf.pool[id].sym == -1) {
    if (blen == 0) {
      blen = 8;
      uint8_t prev = byte;
      jpg.ip.read(reinterpret_cast<char*>(&byte), 1);
      if (prev == 0xFF && byte == 0) {
        jpg.ip.read(reinterpret_cast<char*>(&byte), 1);
      }
    }
    id = huf.walk(byte, blen, id);
  }
  return huf.pool[id].sym;
}

uint16_t retrieve(uint8_t &byte, int &blen, uint8_t len, JPEGData &jpg) {
  uint16_t res = 0;
  while (len > blen) {
    res = (res << blen) | (byte & ((1 << (blen)) - 1));
    len -= blen;
    blen = 8;
    uint8_t prev = byte;
    jpg.ip.read(reinterpret_cast<char*>(&byte), 1);
    if (prev == 0xFF && byte == 0) {
      jpg.ip.read(reinterpret_cast<char*>(&byte), 1);
    }
  }
  res = (res << len) | ((byte >> (blen - len)) & ((1 << len) - 1));
  blen -= len;
  return res;
}

int16_t extend(uint16_t diff, uint8_t len) {
  int16_t res = diff;
  if (res < (1<<(len-1))) res -= (1<<len) - 1;
  return res;
}

void decode_block(int16_t b_zz[], int cid, int &predDC, uint8_t &byte, int &blen, JPEGData &jpg) {
  memset(b_zz, 0, sizeof(int16_t[64]));
  uint8_t len = decode(jpg.huf[0][jpg.comp[cid].td], byte, blen, jpg);
  b_zz[0] = predDC + extend(retrieve(byte, blen, len, jpg), len);
  predDC = b_zz[0];
  for (int i=1; i<64; i++) {
    len = decode(jpg.huf[1][jpg.comp[cid].ta], byte, blen, jpg);
    if (len == 0) break;
    uint8_t run = (len >> 4) & 0xF, size = (len) & 0xF;
    i += run;
    b_zz[i] = extend(retrieve(byte, blen, size, jpg), size);
  }

#ifdef VERBOSE
  cerr<<"b_zz =";
  for (int i=0; i<64; i++)
    cerr<<" "<<b_zz[i];
  cerr<<endl;
#endif

  for (int i=0; i<64; i++) {
    b_zz[i] = b_zz[i] * jpg.qt[jpg.comp[cid].tq][i];
  }

#ifdef VERBOSE
  cerr<<"b_zz =";
  for (int i=0; i<64; i++)
    cerr<<" "<<b_zz[i];
  cerr<<endl;
#endif

}
void scan(JPEGData &jpg) {
#ifdef VERBOSE
  cerr<<"Start scanning"<<endl;
#endif
  uint8_t byte = 0;
  int blen = 0;

  int cnt_h = (jpg.width + jpg.maxH * 8 - 1) / (jpg.maxH * 8),
      cnt_v = (jpg.height + jpg.maxV * 8 - 1) / (jpg.maxV * 8);

#ifdef VERBOSE
  cerr<<"cnt_v = "<<cnt_v<<", cnt_h = "<<cnt_h<<endl;
#endif

  vector<int> predDC(jpg.nComp());
  fill(predDC.begin(), predDC.end(), 0);

  for (int v=0; v<cnt_v; v++) {
    for (int h=0; h<cnt_h; h++) {
      for (int cid=0; cid<jpg.nComp(); cid++) {
        int V = jpg.comp[cid].v;
        int H = jpg.comp[cid].h;
        for (int j=0; j<V; j++) {
          for (int k=0; k<H; k++) {
            int16_t b_zz[64];
            decode_block(b_zz, cid, predDC[cid], byte, blen, jpg);
#ifdef VERBOSE
            fprintf(stderr, "====\n");
            fprintf(stderr, "Component %d, block (%d, %d)\n", cid, V * v + j, H * h + k);
#endif
            int16_t mat[8][8];
            zigzag(b_zz, mat);

            double fmat[8][8];
            for (int x=0; x<8; x++) {
              for (int y=0; y<8; y++) fmat[x][y] = mat[x][y];
            }
            idct8x8(fmat);
            for (int x=0; x<8; x++) {
              for (int y=0; y<8; y++) mat[x][y] = round(fmat[x][y]);
            }

#ifdef VERBOSE
            for (int x=0; x<8; x++) {
              for (int y=0; y<8; y++)
                cerr<<mat[x][y]<<" \n"[y==7];
            }
#endif

            for (int icnt=0, i=V*v*8+j*8; icnt<8 and i<jpg.comp[cid].height; icnt++, i++) {
              for (int jcnt=0, j=H*h*8+k*8; jcnt<8 and j<jpg.comp[cid].width; jcnt++, j++)
                jpg.YCbCr[cid][i][j] = max(min(mat[icnt][jcnt] + 128, 255), 0);
            }
          }
        }
      }
    }
  }
}

void output(JPEGData &jpg) {
  int height = jpg.height;
  int width = jpg.width;
  bitmap_image bmp(width, height);

  int scale_Cb_x = jpg.maxV / jpg.comp[1].v,
      scale_Cb_y = jpg.maxH / jpg.comp[1].h,
      scale_Cr_x = jpg.maxV / jpg.comp[2].v,
      scale_Cr_y = jpg.maxH / jpg.comp[2].h;
  for (int x=0; x<height; x++) {
    for (int y=0; y<width; y++) {
      double Y = jpg.YCbCr[0][x][y];
      double Cb = jpg.YCbCr[1][min(x/scale_Cb_x, jpg.comp[1].height-1)][min(y/scale_Cb_y, jpg.comp[1].width-1)];
      double Cr = jpg.YCbCr[2][min(x/scale_Cr_x, jpg.comp[1].height-1)][min(y/scale_Cr_y, jpg.comp[2].width-1)];
      int R = round(Y + 1.402 * (Cr - 128.0)),
          G = round(Y - 0.34414 * (Cb - 128.0) - 0.71414 * (Cr - 128.0)),
          B = round(Y + 1.772 * (Cb - 128.0));
      R = max(min(R, 255), 0);
      G = max(min(G, 255), 0);
      B = max(min(B, 255), 0);
      rgb_t c = make_colour(R, G, B);
      bmp.set_pixel(y, x, c);
    }
  }

  bmp.save_image("output.bmp");
}

bool parse_SOS(const Segment &seg, JPEGData &jpg) {
  int pos = 0;
  int Ns = seg.buf[pos++];
  assert(Ns == jpg.nComp());
  for (int i=0; i<Ns; i++) {
    assert(jpg.comp[i].c == seg.buf[pos++]);
    jpg.comp[i].td = (seg.buf[pos] >> 4) & 0xF;
    jpg.comp[i].ta = (seg.buf[pos++]) & 0xF;
#ifdef VERBOSE
    cerr<<"Component "<<i<<": Cs = "<<(int)jpg.comp[i].c<<", Td = "<<(int)jpg.comp[i].td<<", Ta = "<<(int)jpg.comp[i].ta<<endl;
#endif
  }
  int Ss = seg.buf[pos++];
  int Se = seg.buf[pos++];
  int Ah = (seg.buf[pos] >> 4) & 0xF;
  int Al = (seg.buf[pos++]) & 0xF;
  assert(Ss == 0);
  assert(Se == 63);
  assert(Ah == 0);
  assert(Al == 0);

  scan(jpg);
  output(jpg);

  return true;
}

