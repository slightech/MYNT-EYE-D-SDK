
.PHONY: samples
samples:
	@cd samples/; \
	mkdir -p build; \
	cd build/; \
	cmake ..; \
	make

.PHONY: ros
ros:
	@cd wrappers/ros/; \
	catkin_make
