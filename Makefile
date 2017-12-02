include CommonDefs.mk

.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "Usage:"
	@echo "  make help"
	@echo "  make all                       build all"
	@echo "  make ros                       build ros wrapper"
	@echo "  make pkg                       package sdk"
	@echo "  make clean                     clean"
	@echo "  make cleanall                  clean all"
	@echo "Usage More:"
	@echo "  make host                      display variables"
	@echo "  make [modules|samples|sdk]     build a specified part"
	@echo "  make [cleanros|cleanpkg]       clean a specified part"

.PHONY: all
all: sdk samples

.PHONY: clean
clean:
	@$(call echo,Make $@ ...)
	@$(call rm,./output/)
	@$(call rm,./modules/build/)
	@$(call rm,./samples/build/)
	@$(call rm,./pkginfo.sh)
	@$(FIND) . -type f -name ".DS_Store" -print0 | xargs -0 rm -f

.PHONY: cleanall
cleanall: clean cleanros cleanpkg
	@$(call echo,Make $@ ...)
	@$(call rm_f,build-*,./apps/)

.PHONY: host
host:
	@$(call echo,Make $@ ...)
	@echo HOST_OS: $(HOST_OS)
	@echo HOST_ARCH: $(HOST_ARCH)
	@echo HOST_NAME: $(HOST_NAME)
	@echo ECHO: $(ECHO)
	@echo CC: $(CC)
	@echo CXX: $(CXX)
	@echo MAKE: $(MAKE)
	@echo BUILD: $(BUILD)
	@echo CMAKE: $(CMAKE)
	@echo LDD: $(LDD)
	@echo FIND: $(FIND)
	@echo PKGNAME: $(PKGNAME)

.PHONY: modules
modules:
	@$(call echo,Make $@ ...)
	@$(call cmake_build,./modules/build)

.PHONY: samples
samples: modules
	@$(call echo,Make $@ ...)
	@$(call cmake_build,./samples/build)

.PHONY: sdk
sdk: modules include

.PHONY: include
include:
	@$(call echo,Make $@ ...)
	@$(call mkdir,./output/include)
	@$(call echo,Copy ./modules/core/include/api to ./output/include ...,1;35)
	@$(call cp,./modules/core/include/api,./output/include)

.PHONY: ros
ros: sdk
	@$(call echo,Make $@ ...)
	@cd ./wrappers/ros && catkin_make

.PHONY: cleanros
cleanros:
	@$(call echo,Make $@ ...)
	@$(call rm,./wrappers/ros/build/)
	@$(call rm,./wrappers/ros/devel/)
	@$(call rm,./wrappers/ros/install/)
	@$(call rm,./wrappers/ros/.catkin_workspace)
	@$(call rm,./wrappers/ros/src/CMakeLists.txt)

.PHONY: pkg
pkg: sdk cleanros
	@$(call echo,Make $@ ...)

	@$(call echo,Copy ./cmake to ./output/cmake ...,1;35)
	@$(call cp,./cmake/Utils.cmake,./output/cmake/Utils.cmake)
	@$(call cp,./cmake/Common.cmake,./output/cmake/Common.cmake)
	@$(call cp,./cmake/DetectCXX11.cmake,./output/cmake/DetectCXX11.cmake)

	@$(call echo,Copy ./samples to ./output/samples ...,1;35)
	@$(call rm,./samples/build/)
	@./scripts/_cp_subs.sh ./samples ./output/samples

	@$(call echo,Copy ./wrappers to ./output/wrappers ...,1;35)
	@./scripts/_cp_subs.sh ./wrappers ./output/wrappers

	@$(call echo,Copy ./platforms/? to ./output ...,1;35)
	@$(call get_platform_path,./platforms); \
	if [ -n "$${plat_path}" ]; then \
		$(ECHO) "Copy $${plat_path} to ./output ..."; \
		./scripts/_cp_subs.sh $${plat_path} ./output; \
	fi

	@$(shell sh ./pkginfo.sh); dst=$(PKGNAME)-opencv-$$OpenCV_VERSION; \
	$(call echo,Copy ./output to $$dst ...,1;35); \
	$(call rm,$$dst); $(ECHO) "CP: ./output > $$dst"; cp -Rp "./output/." "$$dst"; \
	rm -f $$dst/tools/*.bat; \
	$(call echo,Compress $$dst.tar.gz ...,1;35); \
	tar -zcf $$dst.tar.gz $$dst; \
	$(call echo,Compress $$dst.tar.gz done,1;35)

.PHONY: cleanpkg
cleanpkg:
	@$(call echo,Make $@ ...)
	@$(call rm_f,$(PKGNAME)*)
