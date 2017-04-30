#include "huffman.h"
#include <cassert>

Huffman::Node::Node() {
 sym = -1;
 ch[0] = ch[1] = 0; 
}

int Huffman::get_node() {
  pool[mem] = Node();
  return mem++;
}

Huffman::Huffman() {
  mem = 1;
  root = get_node();
}

void Huffman::insert(int code, int len, int sym) {
  int id = root;
  for (int i=len-1; i>=0; i--) {
    int b = (code >> i) & 1;
    int &ch = pool[id].ch[b];
    if (!ch) ch = get_node();
    id = ch;
  }
  pool[id].sym = sym;
}

int Huffman::walk(int v, int &k, int id) {
  while (k > 0 and pool[id].sym == -1) {
    k--;
    id = pool[id].ch[(v>>k)&1];
    assert(id);
  }
  return id;
}
