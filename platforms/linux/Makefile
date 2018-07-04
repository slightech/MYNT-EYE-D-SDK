
.PHONY: clean
clean:
	-rm -rf ./samples/build/

.PHONY: cleanall
cleanall: clean cleanros

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

.PHONY: cleanros
cleanros:
	-rm -rf ./wrappers/ros/build/
	-rm -rf ./wrappers/ros/devel/
	-rm -rf ./wrappers/ros/install/
	-rm -rf ./wrappers/ros/.catkin_workspace
	-rm -rf ./wrappers/ros/src/CMakeLists.txt
