include ../common.mk

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
     $(CFLAGS_MATH) \
     $(CFLAGS_COMMON) \
     $(CFLAGS_LCMTYPES) \
     $(CFLAGS_GSL) \
     -O2 -w
LDFLAGS  = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	  $(LDFLAGS_IMAGESOURCE) \
	  $(LDFLAGS_MATH) \
	  $(LDFLAGS_COMMON) \
		$(LDFLAGS_LCMTYPES) \
		$(LDFLAGS_GSL)

#LIB_MAEBOT_DATA = $(LIB_PATH)/libmaebotdata.a
#LIB_MAEBOT_DATA_OBJ = maebot_data.o

#LIB_LASER_MATCHER = $(LIB_PATH)/lasermatcher.a
#LIB_LASER_MATCHER_OBJ = laser_matcher.o

#LIB_OCCUPANCY_MAP = $(LIB_PATH)/occupancymap.a
#LIB_OCCUPANCY_MAP_OBJ = occupancy_map.o


LIB_MAPPING = $(LIB_PATH)/libmapping.a


LIB_A1 = $(LIB_PATH)/liba1.a
LIB_A1_OBJ = occupancy_map.o action_model.o particle_data.o maebot_data.o sensor_model.o odometry_matcher.o path_planning.o 
#pose_tracker.o 

LIBDEPS  = $(call libdeps, $(LDFLAGS))

BIN_LCM_HANDLE_TEST = $(BIN_PATH)/lcm_handle_test
BIN_MAEBOT_LOCALIZATION = $(BIN_PATH)/maebot_localization
BIN_MAEBOT_LOCALIZATION_ERROR = $(BIN_PATH)/maebot_localization_error
BIN_MAEBOT_SLAM = $(BIN_PATH)/maebot_slam
BIN_PATH_PLANNING_TEST = $(BIN_PATH)/path_planning_test
BIN_MAEBOT_OCCUPANCY_MAP = $(BIN_PATH)/maebot_occupancy_map
ALL = $(LIB_A1) $(BIN_MAEBOT_LOCALIZATION) $(BIN_MAEBOT_SLAM) $(BIN_PATH_PLANNING_TEST)

all: $(ALL)

$(LIB_A1): $(LIB_A1_OBJ) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

$(BIN_LCM_HANDLE_TEST): lcm_handle_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_LOCALIZATION): maebot_localization.o $(LIBDEPS) $(LIB_A1) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_LOCALIZATION_ERROR): maebot_localization_error.o $(LIBDEPS) $(LIB_A1) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MAEBOT_SLAM): maebot_slam.o $(LIBDEPS) $(LIB_A1) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_PATH_PLANNING_TEST): path_planning_test.o $(LIBDEPS) $(LIB_A1) $(LIB_MAPPING)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
