/** \mainpage

  Supervision of data transmission between HPP and SoT.
  It implements a finite state machine using Smach. There are three states:
  \li agimus.path_execution.play_path.WaitForInput
  \li agimus.path_execution.play_path.InitializePath (Init)
  \li agimus.path_execution.play_path.PlayPath (Play)

  \image html figures/smach-viewer-inkscape.svg "Finite state machine."

  \todo Split file \c src/agimus/path_execution/play_path.py into 3 files with one class per file.

*/
