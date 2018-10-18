# Copyright 2018 Slightech Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MKFILE_DIR := $(patsubst %/,%,$(dir $(MKFILE_PATH)))

include CommonDefs.mk

.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "Usage:"
	@echo "  make help      show help message"
	@echo "  make init      init project"
	@echo "  make build     build project"
	@echo "  make install   build and install"
	@echo "  make samples   build samples"
	@echo "  make tools     build tools"
	@echo "  make ros       build ros wrapper"
	@echo "  make apidoc    build api doc"
	@echo "  make pkg       package sdk"
	@echo "  make clean     clean"
	@echo "  make cleanall  cleanall"

# deps

submodules:
	@git submodule update --init

# init

init: submodules
	@$(call echo,Make $@)
	@$(SH) ./scripts/init.sh

.PHONY: init

# build

build:
	@$(call echo,Make $@)
	@$(call cmake_build,./_build,..,-DCMAKE_INSTALL_PREFIX=$(MKFILE_DIR)/_install)

.PHONY: build

# install

install: build
	@$(call echo,Make $@)
	@$(call make_install,./_build)

.PHONY: install

# samples

samples: install
	@$(call echo,Make $@)
	@$(call cmake_build,./samples/_build)

all: samples

.PHONY: all samples

# tools

tools: install
	@$(call echo,Make $@)
	@$(call cmake_build,./tools/_build)

.PHONY: tools

# ros

.PHONY: ros
ros: install
	@$(call echo,Make $@)
	@cd ./wrappers/ros && catkin_make

.PHONY: cleanros
cleanros:
	@$(call echo,Make $@)
	@$(call rm,./wrappers/ros/build/)
	@$(call rm,./wrappers/ros/devel/)
	@$(call rm,./wrappers/ros/install/)
	@$(call rm,./wrappers/ros/.catkin_workspace)
	@$(call rm,./wrappers/ros/src/CMakeLists.txt)

# doc

apidoc:
	@$(call echo,Make $@)
	@$(SH) ./docs/build.sh

opendoc: apidoc
	@$(call echo,Make $@)
	@$(shell $(SH) ./docs/langs.sh 1); \
	for lang in "$${LANGS[@]}"; do \
		html=./docs/_output/$$lang/html/index.html; \
		[ -f "$$html" ] && $(SH) ./scripts/open.sh $$html; \
	done

cleandoc:
	@$(call rm,./docs/_output/)

.PHONY: apidoc opendoc cleandoc

# pkg

.PHONY: pkg
pkg: install cleanros
	@$(call echo,Make $@)

	@$(call echo,Copy ./cmake to ./_install/cmake ...,1;35)
	@$(call cp,./cmake/Common.cmake,./_install/cmake/Common.cmake)
	@$(call cp,./cmake/DetectCXX11.cmake,./_install/cmake/DetectCXX11.cmake)
	@$(call cp,./cmake/DetectOpenCV.cmake,./_install/cmake/DetectOpenCV.cmake)
	@$(call cp,./cmake/IncludeGuard.cmake,./_install/cmake/IncludeGuard.cmake)

	@$(call echo,Copy ./samples to ./_install/samples ...,1;35)
	@$(call rm,./samples/_build/)
	@$(call rm,./samples/_output/)
	@./scripts/_cp_subs.sh ./samples ./_install/samples

	@$(call echo,Copy ./wrappers to ./_install/wrappers ...,1;35)
	@./scripts/_cp_subs.sh ./wrappers ./_install/wrappers

	@$(call echo,Copy ./platforms/? to ./_install ...,1;35)
	@$(call get_platform_path,./platforms); \
	if [ -n "$${plat_path}" ]; then \
		$(ECHO) "Copy $${plat_path} to ./_install ..."; \
		./scripts/_cp_subs.sh $${plat_path} ./_install; \
	fi

	@$(shell sh ./pkginfo.sh); dst=$(PKGNAME)-opencv-$$OpenCV_VERSION; \
	$(call echo,Copy ./_install to $$dst ...,1;35); \
	$(call rm,$$dst); $(ECHO) "CP: ./_install > $$dst"; cp -Rp "./_install/." "$$dst"; \
	rm -f $$dst/tools/*.bat; \
	$(call echo,Compress $$dst.tar.gz ...,1;35); \
	tar -zcf $$dst.tar.gz $$dst; \
	$(call echo,Compress $$dst.tar.gz done,1;35)

.PHONY: cleanpkg
cleanpkg:
	@$(call echo,Make $@)
	@$(call rm_f,$(PKGNAME)*)

# clean

.PHONY: clean
clean:
	@$(call echo,Make $@)
	@$(call rm,./_build/)
	@$(call rm,./_output/)
	@$(call rm,./_install/)
	@$(call rm,./samples/_build/)
	@$(call rm,./samples/_output/)
	@$(call rm,./tools/_build/)
	@$(call rm,./tools/_output/)
	@$(call rm,./pkginfo.sh)
	@$(FIND) . -type f -name ".DS_Store" -print0 | xargs -0 rm -f

.PHONY: cleanall
cleanall: clean cleanros cleandoc cleanpkg
	@$(call echo,Make $@)
	@$(call rm_f,build-*,./apps/)

.PHONY: host
host:
	@$(call echo,Make $@)
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
