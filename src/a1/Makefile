include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
     $(CFLAGS_MATH) \
     $(CFLAGS_COMMON) \
     $(CFLAGS_LCMTYPES) \
     -O2
LDFLAGS  = $(LDFLAGS_STD) \
		$(LDFLAGS_LCMTYPES)
LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_LCM_HANDLE_TEST = $(BIN_PATH)/lcm_handle_test

ALL = $(BIN_LCM_HANDLE_TEST)

all: $(ALL)

$(BIN_LCM_HANDLE_TEST): lcm_handle_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
