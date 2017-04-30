CC = g++-4.9
CXX = g++-4.9
CPPFLAGS += -std=c++14 -O2 -Wall -Wno-unused-result
OBJS = jpeg_decoder.o segment.o utils.o huffman.o
EXEC = jpeg_decoder

all: $(EXEC)

$(EXEC): $(OBJS)

clean:
	$(RM) $(EXEC) $(OBJS)
