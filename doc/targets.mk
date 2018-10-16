#
# Copyright (c) 2018 CNRS-LAAS
# Author: Florent Lamiraux, Joseph Mirabel
#

status:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".status; \
	done

log:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".log; \
	done

update:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".update; \
	done

%.checkout:
	@if [ -d $(@:.checkout=) ]; then \
		${MAKE} $(@:.checkout=).status; \
	else \
	  @$(call _status,"Checking out branch ${$(@:.checkout=)_branch} of ${$(@:.checkout=)_repository}/$(@:.checkout=) ... "); \
		git clone ${GIT_QUIET} -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=); \
		cd ${SRC_DIR}/$(@:.checkout=) && git submodule ${GIT_QUIET} update --init; \
	  @echo "${_msg_done}."; \
	fi

%.update:
	if [ "${$(@:.update=)_repository}" = "" ]; then \
		echo "$(@:.update=) is not referenced"; \
	else \
		cd ${SRC_DIR}/$(@:.update=);\
		git remote set-url origin ${$(@:.update=)_repository}/$(@:.update=);\
		git fetch ${GIT_QUIET} origin;\
		git checkout ${GIT_QUIET} -B ${$(@:.update=)_branch} origin/${$(@:.update=)_branch};\
		git submodule ${GIT_QUIET} update; \
	fi

%.configure: %.configure.dep
	${MAKE} $(@:.configure=).configure_nodep

%.configure_nodep:%.checkout
	@$(call _status,"Configuring $(@:.configure_nodep=) ... ")
	@if [ -n "$($(@:.configure_nodep=)_subdirs)" ]; then \
		echo ""; \
		for d in $($(@:.configure_nodep=)_subdirs); do \
			$(call _status,"- $$d: "); \
		  ${MAKE} $(@:.configure_nodep=)___$${d}.run_cmake; \
		done; \
	else \
		${MAKE} $(@:.configure_nodep=).run_cmake; \
	fi

%.run_cmake:
	$(eval curdir := $(subst ___,/,$(@:.run_cmake=)))
	@mkdir -p ${SRC_DIR}/$(curdir)/${BUILD_FOLDER}
	@cd ${SRC_DIR}/$(curdir)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
			-DINSTALL_DOCUMENTATION=${INSTALL_DOCUMENTATION} \
			-DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" \
			${$(@:.run_cmake=)_extra_flags} .. \
			1> configure.stdout.cmake.log \
			2> configure.stderr.cmake.log
	@if [ -s "${SRC_DIR}/$(curdir)/${BUILD_FOLDER}/configure.stderr.cmake.log" ]; then \
	 	echo "${_msg_warn}."; \
	  echo "See logs in ${SRC_DIR}/$(curdir)/${BUILD_FOLDER}/configure.*.cmake.log"; \
	else echo "${_msg_done}."; fi

%.install:%.configure.dep
	@${MAKE} $(@:.install=).install_nodep

%.install_nodep:%.configure_nodep
	@$(call _status,"Installing $(@:.install_nodep=) ... ")
	@if [ -n "$($(@:.install_nodep=)_subdirs)" ]; then \
		echo ""; \
		for d in $($(@:.install_nodep=)_subdirs); do \
			$(call _status,"- $$d: "); \
		  ${MAKE} $(@:.install_nodep=)___$${d}.run_make; \
		done; \
	else \
		${MAKE} $(@:.install_nodep=).run_make; \
	fi

%.run_make:
	$(eval curdir := $(subst ___,/,$(@:.run_make=)))
	@${MAKE} -C ${SRC_DIR}/$(curdir)/${BUILD_FOLDER} install \
		1> ${SRC_DIR}/$(curdir)/${BUILD_FOLDER}/install.stdout.make.log \
		2> ${SRC_DIR}/$(curdir)/${BUILD_FOLDER}/install.stderr.make.log
	@if [ -s "${SRC_DIR}/$(curdir)/${BUILD_FOLDER}/install.stderr.make.log" ]; then \
		echo "${_msg_warn}."; \
		echo "See logs in ${SRC_DIR}/$(curdir)/${BUILD_FOLDER}/install.*.make.log"; \
	else echo "${_msg_done}."; fi

%.uninstall:
	${MAKE} -C ${SRC_DIR}/$(@:.uninstall=)/${BUILD_FOLDER} uninstall

%.clean:
	${MAKE} -C ${SRC_DIR}/$(@:.clean=)/${BUILD_FOLDER} clean

%.very-clean:
	rm -rf ${SRC_DIR}/$(@:.very-clean=)/${BUILD_FOLDER}/*

%.status:
	@cd ${SRC_DIR}/$(@:.status=); \
	echo \
	"\033[1;36m------- Folder $(@:.status=) ---------------\033[0m"; \
	git --no-pager -c status.showUntrackedFiles=no status --short --branch;\

%.log:
	@cd ${SRC_DIR}/$(@:.log=); \
	if [ -f .git/refs/heads/${$(@:.log=)_branch} ]; then \
		echo -n "$(@:.log=): "; \
		cat .git/refs/heads/${$(@:.log=)_branch}; \
	fi

include agimus/doc/dependencies.mk
include agimus/doc/specific-rules.mk
