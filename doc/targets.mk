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
		echo -n "$(@:.checkout=) already checkout out. "; \
	else \
	  echo -n "Checking out branch ${$(@:.checkout=)_branch} of ${$(@:.checkout=)_repository}/$(@:.checkout=) ... "; \
		git clone ${GIT_QUIET} -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=); \
		cd ${SRC_DIR}/$(@:.checkout=) && git submodule ${GIT_QUIET} update --init; \
	fi
	@echo "${_msg_done}."

%.update:
	if [ "${$(@:.update=)_repository}" = "" ]; then \
		echo "$(@:.update=) is not referenced"; \
	else \
		cd ${SRC_DIR}/$(@:.update=);\
		git remote rm origin;\
		git remote add origin ${$(@:.update=)_repository}/$(@:.update=);\
		git fetch origin;\
		git checkout -q --detach;\
		git branch -f ${$(@:.update=)_branch} origin/${$(@:.update=)_branch};\
		git checkout -q ${$(@:.update=)_branch};\
		git submodule update; \
	fi


%.configure: %.configure.dep
	${MAKE} $(@:.configure=).configure_nodep

%.configure_nodep:%.checkout
	@echo -n "Configuring $(@:.configure_nodep=) ... "
	@mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}
	@cd ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
			-DINSTALL_DOCUMENTATION=${INSTALL_DOCUMENTATION} \
			-DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" \
			${$(@:.configure_nodep=)_extra_flags} .. \
			1> configure.stdout.cmake.log \
			2> configure.stderr.cmake.log
	@if [ -s "${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}/configure.stderr.cmake.log" ]; then \
	 	echo "${_msg_warn}."; \
	  echo "See logs in ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}/configure.*.cmake.log"; \
	else echo "${_msg_done}."; fi

%.install:%.configure
	@echo -n "Installing $(@:.install=) ... "
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER} install \
		1> ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER}/install.stdout.make.log \
		2> ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER}/install.stderr.make.log
	@if [ -s "${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER}/install.stderr.make.log" ]; then \
		echo "${_msg_warn}."; \
		echo "See logs in ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER}/install.*.make.log"; \
	else echo "${_msg_done}."; fi

%.install_nodep:%.configure_nodep
	${MAKE} -C ${SRC_DIR}/$(@:.install_nodep=)/${BUILD_FOLDER} install

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
