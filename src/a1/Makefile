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

#LIB_MAEBOT_DATA = $(LIB_PATH)/libmaebotdata.a
#LIB_MAEBOT_DATA_OBJ = maebot_data.o

#LIB_LASER_MATCHER = $(LIB_PATH)/lasermatcher.a
#LIB_LASER_MATCHER_OBJ = laser_matcher.o

#LIB_OCCUPANCY_MAP = $(LIB_PATH)/occupancymap.a
#LIB_OCCUPANCY_MAP_OBJ = occupancy_map.o


LIB_MAPPING = $(LIB_PATH)/libmapping.a


LIB_A1 = $(LIB_PATH)/liba1.a
LIB_A1_OBJ = maebot_data.o laser_matcher.o occupancy_map.o maebot_localization_util.o action_model.o odometry_matcher.o

LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_LCM_HANDLE_TEST = $(BIN_PATH)/lcm_handle_test
BIN_MAEBOT_OCCUPANCY_MAP = $(BIN_PATH)/maebot_occupancy_map

ALL = $(LIB_A1) $(BIN_LCM_HANDLE_TEST) $(BIN_MAEBOT_OCCUPANCY_MAP)

all: $(ALL)

$(LIB_A1): $(LIB_A1_OBJ) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

$(BIN_LCM_HANDLE_TEST): lcm_handle_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_OCCUPANCY_MAP): maebot_occupancy_map.o $(LIBDEPS) $(LIB_A1) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
