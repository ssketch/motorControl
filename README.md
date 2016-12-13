# motorControl
software for all work in the Stanford CHARM Lab pertaining to human motor control (and how it relates to stroke-induced deficits)

What's included:
1) Scripts to compute reaching trajectories, workspaces, and force envelopes (main_*.m).
2) Control, plant, and estimate blocks to compute the model predictive control outputs, integrate the arm state, and sense the resulting change of state, respectively.
3) Include directory with helper functions for the control and estimate blocks including the MPT3 toolbox for model predictive control.
4) Class definitions for a 2-dimensional, 2 degree-of-freedom, RR robot model of the arm and a 3-dimensional, 4 degree-of-freedom, RRRR robot model of the arm.
5) A resource folder to with useful information and functions to help develop and understand the code package.
6) A wiki describing the package and its uses.

_________________________________________
Using Git from the terminal instructions:
Push changes to git:
Step 1:  Navigate to the directory that you're working out of for git
  cd write/folder/path/here
Step 2: Initialize git in the terminal
  git init [paste http://gitlocation]
Step 3: Add files to git
  git add -v --all
Step 4: Commit changes and write a message
  git commit -m "write message here"
Step 5: Push changes to master branch
  git push origin master
  
Pull changes from git:
Step 1:  Navigate to the directory that you're working out of for git
  cd write/folder/path/here
Step 2: Initialize git in the terminal
  git init [paste http://gitlocation]
Step 5: Pull changes from master branch
  git pull origin master

Help:
https://xkcd.com/1597/
