/** \mainpage

  Supervision of data transmission between HPP and SoT.  It implements
  a finite state machine using <a href="https://wiki.ros.org/smach">Smach</a>. The three states are:

  \li \link agimus.path_execution.wait_for_input.WaitForInput
  WaitForInput \endlink
  \li \link agimus.path_execution.initialize_path.InitializePath Init \endlink
  \li \link agimus.path_execution.play_path.PlayPath Play \endlink

  \image html figures/smach-viewer-inkscape.svg "Finite state machine."

*/
