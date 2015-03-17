include ../common.mk

# flags for building the gtk library
CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GSL) \
    -g

CFLAGS = $(CFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_GSL) \
    -g

LDFLAGS = $(LDFLAGS_STD) \
	  $(LDFLAGS_VX_GL) \
	  $(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
	  $(LDFLAGS_LCMTYPES) \
	  $(LDFLAGS_VX) \
	  $(LDFLAGS_GTK) \
	  $(LDFLAGS_USB) \
	  $(LDFLAGS_PNG) \
	  $(LDFLAGS_DC1394) \
	  $(LDFLAGS_MAPPING) \
      $(LDFLAGS_GSL)

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o

BIN_IP = $(BIN_PATH)/ip
BIN_BLOB = $(BIN_PATH)/blob
BIN_IMAGE_PROCESSOR = $(BIN_PATH)/a2_image_to_arm_coord
BIN_BOARD_TEST = $(BIN_PATH)/a2_board_test
BIN_TIC_TAC_TOE = $(BIN_PATH)/tic_tac_toe
#BIN_INVERSE_KINEMATICS = $(BIN_PATH)/a2_inverse_kinematics

ALL = $(LIB_EECS467) $(BIN_IP) $(BIN_IMAGE_PROCESSOR) \
	  $(BIN_TIC_TAC_TOE) $(BIN_BLOB) $(BIN_BOARD_TEST)

all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_IP): ip.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_BLOB): blob.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_IMAGE_PROCESSOR): ImageProcessor.o driver.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_BOARD_TEST): ImageProcessor.o Board.o board_test.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_INVERSE_KINEMATICS): Arm.o test_5.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_TIC_TAC_TOE): tic_tac_toe.o Arm.o ImageProcessor.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
