robotpkg_branch=master
robotpkg_repository=git://git.openrobots.org/robots

robotpkg-wip_branch=master
robotpkg-wip_repository=git://git.openrobots.org/robots/robotpkg

robotpkg.configure.dep:
robotpkg-wip.configure.dep: robotpkg.configure

robotpkg.configure_nodep: robotpkg.checkout
	$(call _status, "Configuring $(@:.configure_nodep=) ... ")
	@test -f ${INSTALL_DIR}/etc/robotpkg.conf || (cd ${SRC_DIR}/$(@:.configure_nodep=)/bootstrap; \
		./bootstrap --prefix=${INSTALL_DIR})
	@if ! grep -q "# --- Agimus configuration ----" ${INSTALL_DIR}/etc/robotpkg.conf; then \
		echo ""                                                  >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo ""                                                  >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "# --- Agimus configuration ----------------------" >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "JRL_FTP_USER=jrl"                                  >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "JRL_FTP_PASSWD=l445jrl"                            >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo ""                                                  >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "PREFER_ALTERNATIVE.c-compiler=ccache-gcc gcc"      >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "PREFER_ALTERNATIVE.c++-compiler=ccache-g++ g++"    >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "# For ccache"                                      >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "HOME.env=/local/jmirabel/devel/openrobots/install" >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo ""                                                  >> ${INSTALL_DIR}/etc/robotpkg.conf; \
		echo "ACCEPTABLE_LICENSES+=pal-license"                  >> ${INSTALL_DIR}/etc/robotpkg.conf; \
	fi
	@echo "${_msg_done}."

robotpkg-wip.checkout:
	$(call _status, "Checking out branch ${$(@:.checkout=)_branch} of ${$(@:.checkout=)_repository}/$(@:.checkout=) ... ")
	@if [ -d robotpkg/wip ]; then \
		$(call _status, "already "); \
	else \
    git clone ${GIT_QUIET} -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=) ${SRC_DIR}/robotpkg/wip; \
	fi
	@echo "${_msg_done}."

robotpkg-wip.configure_nodep: robotpkg-wip.checkout
	$(call _status, "Configuring $(@:.configure_nodep=) ... ")
	@echo "${_msg_done}."

ifeq (${PAL_DISTRO}, TRUE)
robotpkg.install_nodep:
	$(call _warning, "robotpkg must be compile manually for now")
robotpkg-wip.install_nodep:
	$(call _warning, "robotpkg/wip must be compile manually for now")

else # Ubuntu 16.04

robotpkg.install_nodep: robotpkg.configure_nodep
	#$(call _warning, "robotpkg must be compile manually for now")
	#@${MAKE} -C ${SRC_DIR}/robotpkg/devel/ros-catkin install
	#@${MAKE} -C ${SRC_DIR}/robotpkg/motion/ros-control-toolbox install

robotpkg-wip.install_nodep: robotpkg-wip.configure_nodep
	$(call _warning, "robotpkg/wip must be compile manually for now")
	@cd ${SRC_DIR}/robotpkg/wip/prf-ros-controllers; ${MAKE} install
	#@${MAKE} -C ${SRC_DIR}/robotpkg/wip/ install

endif
