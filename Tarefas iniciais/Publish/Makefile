
include ../../Makefile.conf

# Module name and description
MODULE_NAME = Publish Example
MODULE_COMMENT = Publish Example

LINK ?= g++
CXXFLAGS += -std=c++11

IFLAGS += -I/usr/include/python3.5

LFLAGS += -lglobal -lipc -lparam_interface


#SOURCES = publish_pessoa.c 
#PUBLIC_INCLUDES = publish_example_pessoa.h

SOURCES = publish_example_ackerman.c 
PUBLIC_INCLUDES = publish_example_pessoa.h

# Public libraries, linked to 'astro/lib'.


PUBLIC_BINARIES = publish
TARGETS = publish

publish: publish_example_ackerman.o
#publish: publish_pessoa.o
include ../../Makefile.rules

