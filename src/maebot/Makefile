include ../common.mk

CFLAGS  =  $(CFLAGS_STD) $(CFLAGS_VX) $(CFLAGS_VX_GTK) $(CFLAGS_GTK) $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O4
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_VX) $(LDFLAGS_VX_GTK) $(LDFLAGS_GTK) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES)
LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_EECS467 = $(LIB_PATH)/libeecs467.a

BIN_MAEBOT_DRIVER = $(BIN_PATH)/maebot_driver
BIN_RPLIDAR_DRIVER = $(BIN_PATH)/maebot_rplidar_driver
BIN_LED_TEST = $(BIN_PATH)/maebot_led_test
BIN_LASER_TEST = $(BIN_PATH)/maebot_laser_test
BIN_MOTOR_FEEDBACK_TEST = $(BIN_PATH)/maebot_motor_feedback_test
BIN_DIFF_DRIVE_TEST = $(BIN_PATH)/maebot_diff_drive_test
BIN_SENSOR_DATA_TEST = $(BIN_PATH)/maebot_sensor_data_test
BIN_PROJECT0 = $(BIN_PATH)/project0
BIN_LIDAR_SCAN = $(BIN_PATH)/lidar_scan
BIN_ODO_TRACE = $(BIN_PATH)/odo_trace


ALL = $(BIN_ODO_TRACE)  $(LIB_EECS467) $(BIN_MAEBOT_DRIVER) $(BIN_LED_TEST) $(BIN_LASER_TEST) \
      $(BIN_MOTOR_FEEDBACK_TEST) $(BIN_DIFF_DRIVE_TEST) $(BIN_SENSOR_DATA_TEST) \
      $(BIN_RPLIDAR_DRIVER) $(BIN_PROJECT0)

all: $(ALL)

$(LIB_EECS467): ../apps/eecs467_util.o $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ ../apps/eecs467_util.o

$(BIN_MAEBOT_DRIVER): maebot_driver.o types.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_RPLIDAR_DRIVER): rplidar_driver.o rplidar.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_LED_TEST): led_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_LASER_TEST): laser_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_MOTOR_FEEDBACK_TEST): motor_feedback_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_DIFF_DRIVE_TEST): diff_drive_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_SENSOR_DATA_TEST): sensor_data_test.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_PROJECT0): project0.o rplidar.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) 

$(BIN_ODO_TRACE): odo_trace.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) 

$(BIN_LIDAR_SCAN): lidar_scan.o rplidar.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)

