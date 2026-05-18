
include ../../Makefile.conf

# Module name and description
MODULE_NAME = Subscribe ACKERMAN
MODULE_COMMENT = Subscribe ACKERMAN

LINK ?= g++
CXXFLAGS += -std=c++11

IFLAGS += -I/usr/include/python3.5

LFLAGS += -lglobal -lipc -lparam_interface -lpthread -lbase_ackerman_interface

SOURCES = subscribe_example.c  gnuplot.c
PUBLIC_INCLUDES = publish_example_messages.h
#SOURCES = subscribe_pessoa.c 
#PUBLIC_INCLUDES = publish_example_pessoa.h

# Public libraries, linked to 'astro/lib'.

#PUBLIC_BINARIES = subscribe
#TARGETS = subscribe

PUBLIC_BINARIES = breno
TARGETS = breno

#pessoa: subscribe_pessoa.o
breno: subscribe_example.o gnuplot.o

include ../../Makefile.rules

