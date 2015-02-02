include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
     $(CFLAGS_MATH) \
     $(CFLAGS_COMMON) \
     $(CFLAGS_LCMTYPES) \
     -O2 -w
LDFLAGS  = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
		$(LDFLAGS_LCMTYPES)

LIB_MAEBOT_DATA = $(LIB_PATH)/libmaebotdata.a
LIB_MAEBOT_DATA_OBJ = maebot_data.o

LIB_MAPPING = $(LIB_PATH)/libmapping.a

LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_LCM_HANDLE_TEST = $(BIN_PATH)/lcm_handle_test
BIN_MAEBOT_VX_A1 = $(BIN_PATH)/maebot_vx_a1

ALL = $(LIB_MAEBOT_DATA) $(BIN_LCM_HANDLE_TEST) $(BIN_MAEBOT_VX_A1)

all: $(ALL)

$(LIB_MAEBOT_DATA): $(LIB_MAEBOT_DATA_OBJ) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

$(BIN_LCM_HANDLE_TEST): lcm_handle_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_VX_A1): maebot_vx_a1.o $(LIBDEPS) $(LIB_MAEBOT_DATA) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
