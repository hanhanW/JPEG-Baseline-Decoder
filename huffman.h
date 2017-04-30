#ifndef __HUFFMAN_H_
#define __HUFFMAN_H_
struct Huffman {
  static const int MEM = 65536;
  struct Node {
    int sym, ch[2];
    Node ();
  } pool[MEM];

  int mem,root;

  Huffman ();
  int get_node();
  void insert(int code, int len, int sym);
  int walk(int v, int &k, int id);

};
#endif
