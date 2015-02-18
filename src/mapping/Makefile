include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) -O3 -DNDEBUG
LDFLAGS  = $(LDFLAGS_STD)
LIBDEPS  = $(call libdeps, $(LDFLAGS))

LIB_MAPPING = $(LIB_PATH)/libmapping.a

LIBMAPPING_OBJS = \
	occupancy_grid.o

all: $(LIB_MAPPING)

$(LIB_MAPPING): $(LIBMAPPING_OBJS) $(LIBDEPS) $(LIB_MAEBOT_DATA)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(LIB_COMMON)
