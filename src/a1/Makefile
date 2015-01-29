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
LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_LCM_HANDLE_TEST = $(BIN_PATH)/lcm_handle_test
BIN_MAEBOT_VX_A0 = $(BIN_PATH)/maebot_vx_a0

ALL = $(BIN_LCM_HANDLE_TEST) $(BIN_MAEBOT_VX_A0)

all: $(ALL)

$(BIN_LCM_HANDLE_TEST): lcm_handle_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_VX_A0): maebot_vx_a0.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
