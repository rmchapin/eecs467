include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	 $(CFLAGS_VX_GTK) \
         $(CFLAGS_IMAGESOURCE) \
	 $(CFLAGS_MATH) \
	 $(CFLAGS_COMMON) \
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

BIN_IMAGE_PROCESSOR = $(BIN_PATH)/ip
#BIN_EECS467_REXARM_EXAMPLE = $(BIN_PATH)/eecs467_rexarm_example
#BIN_EECS467_MAEBOT_EXAMPLE = $(BIN_PATH)/eecs467_maebot_example
#BIN_EECS467_MAEBOT_VIDEO_TELOP = $(BIN_PATH)/eecs467_maebot_video_telop

ALL = $(LIB_EECS467) $(BIN_IMAGE_PROCESSOR) 
#$(BIN_EECS467_REXARM_EXAMPLE) \
#      $(BIN_EECS467_MAEBOT_EXAMPLE) $(BIN_EECS467_MAEBOT_VIDEO_TELOP)

all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_IMAGE_PROCESSOR): ip.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

#$(BIN_EECS467_REXARM_EXAMPLE): rexarm_example.o $(LIBDEPS)
#	@echo "\t$@"
#	@$(CC) -o $@ $^ $(LDFLAGS)

#$(BIN_EECS467_MAEBOT_EXAMPLE): maebot_example.o $(LIBDEPS)
#	@echo "\t$@"
#	@$(CC) -o $@ $^ $(LDFLAGS)

#$(BIN_EECS467_MAEBOT_VIDEO_TELOP): maebot_video_teleop.o $(LIBDEPS)
#	@echo "\t$@"
#	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
