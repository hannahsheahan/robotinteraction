# robotinteraction
Code for my vBOT human-robot interaction experiments (collab. with James Ingram)

Contains base c++ code developed for testing human-robot interaction motor learning experiments during my PhD. 
Many experiment variations have been derived from these basic code sets, which are not included in this repo.

In general:
- the code of interest for running the robot is the single .cpp file per directory.
- several configuration files are specified for each main .cpp robot experiment paradigm.
- the m.bat batch file is used for parsing which configuration to use and the savefile to store the recorded interaction data 
   e.g.  m FT-Imagine.cfg testsavefile
