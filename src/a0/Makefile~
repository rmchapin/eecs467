include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 -O2

CXXFLAGS = $(CXXFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CXXFLAGS_IMAGESOURCE) \
	 $(CXXFLAGS_MATH) \
	 $(CXXFLAGS_COMMON) \
	 $(CFLAGS_LCMTYPES) \
	 -O2

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES)

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o

BIN_EECS467_GUI_EXAMPLE = $(BIN_PATH)/eecs467_gui_example
BIN_EECS467_REXARM_EXAMPLE = $(BIN_PATH)/eecs467_rexarm_example
BIN_EECS467_MAEBOT_EXAMPLE = $(BIN_PATH)/eecs467_maebot_example
BIN_EECS467_MAEBOT_VIDEO_TELOP = $(BIN_PATH)/eecs467_maebot_video_telop
BIN_EECS467_MAEBOT_A0 = $(BIN_PATH)/eecs467_maebot_a0
BIN_EECS467_REXARM_SEQUENCE_TEST = $(BIN_PATH)/eecs467_rexarm_sequence_test
BIN_EECS467_REX_VX = $(BIN_PATH)/eecs467_rex_vx

ALL = $(LIB_EECS467) $(BIN_EECS467_GUI_EXAMPLE) $(BIN_EECS467_REXARM_EXAMPLE) \
      $(BIN_EECS467_MAEBOT_EXAMPLE) $(BIN_EECS467_MAEBOT_VIDEO_TELOP) \
      $(BIN_EECS467_MAEBOT_A0) $(BIN_EECS467_REXARM_SEQUENCE_TEST) $(BIN_EECS467_REX_VX)

all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_EECS467_GUI_EXAMPLE): gui_example.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_EECS467_REXARM_EXAMPLE): rexarm_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_EXAMPLE): maebot_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_VIDEO_TELOP): maebot_video_teleop.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_A0): maebot_a0.o $(LIBDEPS)
	@echo "\t$@"
	@$(CPP) -o $@ $^ $(LDFLAGS) $(CFLAGS)

$(BIN_EECS467_REXARM_SEQUENCE_TEST): rexarm_sequence_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_REX_VX): vx_rexarm_demo.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
