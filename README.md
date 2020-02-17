# robotinteraction
Old code from my vBOT human-robot interaction experiments (collab. with James Ingram).
This is a bit of a mess and doesn't include the essential motor.h header, but is useful for record-keeping.

Contains base c++ code developed for testing human-robot interaction motor learning experiments during my PhD. 
We had lots of slightly different experiment variations derived from these basic code sets, but I haven't bothered to include them all in this repo.

In general:
- the code of interest for running the robot is the single .cpp file per directory.
- several configuration files are specified for each main .cpp robot experiment paradigm.
- the m.bat batch file is used for parsing which configuration to use and the savefile to store the recorded interaction data 
   e.g.  m experiment_configuration.cfg test_savefile
