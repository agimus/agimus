SET (PYTHON_FILE
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("agimus" ${F})
ENDFOREACH()

SET (PYTHON_FILE
    initialize_path.py
    play_path.py
    status.py
    error_state.py
    wait_for_input.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("agimus/path_execution" ${F})
ENDFOREACH()

SET (PYTHON_FILE
    path_execution.py
    simulation.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("rqt_agimus" ${F})
ENDFOREACH()
