SET (PYTHON_FILE
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("agimus" ${F})
ENDFOREACH()

SET (PYTHON_FILE
    play_path.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("agimus/path_execution" ${F})
ENDFOREACH()

SET (PYTHON_FILE
    path_execution.py
    __init__.py)

FOREACH(F ${PYTHON_FILE})
  PYTHON_INSTALL_ON_SITE("rqt_agimus" ${F})
ENDFOREACH()
