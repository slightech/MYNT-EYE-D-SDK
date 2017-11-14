include CommonDefs.mk

.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "Usage:"
	@echo "  make help"
	@echo "  make all                       build all parts"
	@echo "  make clean                     clean"
	@echo "  make cleanall                  clean all"
	@echo "Usage More:"
	@echo "  make host                      display variables"
	@echo "  make [sdk|modules|samples]     build a specified part"

.PHONY: all
all: sdk samples

.PHONY: clean
clean:
	@$(call echo,Make $@ ...)
	@$(call rm,./output/)
	@$(call rm,./modules/build/)
	@$(call rm,./samples/build/)
	@$(FIND) . -type f -name ".DS_Store" -print0 | xargs -0 rm -f

.PHONY: cleanall
cleanall: clean
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
samples:
	@$(call echo,Make $@ ...)
	@$(call cmake_build,./samples/build)

.PHONY: sdk
sdk: modules include

.PHONY: include
include:
	@$(call echo,Make $@ ...)
	@$(call mkdir,./output/include)
	@$(call echo,Copy modules/core/include/api/ ...,1;35)
	@$(call cp,./modules/core/include/api,./output/include)
