# code_2020
Excelsior Team 4243 FRC Code 2020


# STEPS TO COMMIT CODE

1. Open git bash: Right-click in the DIRECTORY LEVEL of the git repository (i.e. Destop/code_2020)
	- Opens a git bash terminal, verify the current branch and directory level (ie. ~Desktop/code_2020 (branch_name))
	
2. Check your changes: 'git status'
	- This shows you all file changes, RED means NOT STAGED, GREEN means STAGED
	
**3. Stage your changes: 'git add .' (use period for ALL changes, or just type the changed file name)**

4. Check your staged code: 'git status'
	- This shows you all STAGED in GREEN.... if any in RED it is NOT STAGED and you need to 'git add....' 
	
**5. Commit the staged code!!!!!: 'git commit -m "message goes here"'**
	- This commits the staged code LOCALLY (not in the cloud yet)
	
6. Optionally check for any additional changes.. 'git status' show show branch is AHEAD of origin/branch 

**7. Push the commits to the cloud!!: 'git push origin branch_name'**

8. Check the status again: 'git status'

# STEPS TO SWITCH BRANCHES
1. Checkout a branch locally AFTER pulling: 'git checkout branch_name'
	- See all local branches with 'git branch'
	
2. If creating NEW BRANCH: 'git checkout -b new_branch_name'

# Additional GIT Notes
1. Get any changes from the cloud: 'git pull origin branch_name'

2. If there's any 'merge conflicts' when pulling or pushing, **ask for help

3. To see COMMIT HISTORY: 'git log'