#!/bin/bash
# Short script to create a new controller for SCL
#
# What this script does:
# 1. Copies the example controller to a new directory
# 2. Adds the new directory to GIT;
# 3. Renames all the variables within the files from Example to <your_name>
# 4. Renames all the files in the directory from Example to <your_name>
# 5. Compiles your new <your_name> application
#
# Format for executing the script:
# $ sh create_new_controller.sh <your_name_all_small_letters> <your_git_branch_name>
#
# Author: Gerald Brantner
# Email: geraldb@stanford.edu

if [ $# -ne 2 ]
then
    echo "Format for executing the script:"
    echo "$ sh create_new_controller.sh <your_name_all_small_letters> <your_git_branch_name>"
    exit 1
fi

echo $1
echo $2

# Setting the command line parameters:
your_name=$1
your_git_branch_name=$2

# 1. Copy the example controller to a new directory
rm -rf "scl_"$your_name"_ctrl"
cp -rf scl_example_ctrl "scl_"$your_name"_ctrl"
cd "scl_"$your_name"_ctrl"
# Cleaning up the temporary files from SCL Example
rm -rf build_* scl_eg_ctrl gmon.out 

# 2. Adds the new directory to GIT;
git stash
git reset HEAD --hard
git checkout master
git branch -D $your_git_branch_name"_ctrl_dev"
git branch $your_git_branch_name"_ctrl_dev"
git checkout $your_git_branch_name"_ctrl_dev"
git add *pp
git add *sh
git add CMakeLists.txt
git commit . -m "Adding new controller: "$your_name

# 3. Renames all the variables within the files from Example to <your_name>
# Define all the variables required
cap_first_your_name=`echo $your_name | sed 's/\([a-z]\)\([a-zA-Z0-9]*\)/\u\1\2/g'`
cap_all_your_name=`echo $your_name | sed 's/\(.*\)/\U\1/'`

# Part a: rename first-capital-letter:
for foo in `find . -name '*'`;
do
cat $foo | sed 's/Example/'$cap_first_your_name'/g' > foo2;
echo Updating $foo;
mv foo2 $foo;
done

# Part b: rename all-capital-letters:

for foo in `find . -name '*'`;
do
cat $foo | sed 's/EXAMPLE/'$cap_all_your_name'/g' > foo2;
echo Updating $foo;
mv foo2 $foo;
done

# Part c: rename all-lower_case-letters:
for foo in `find . -name '*txt'`;
do
cat $foo | sed 's/example/'$your_name'/g' > foo2;
echo Updating $foo;
mv foo2 $foo;
done

for foo in `find . -name '*txt'`;
do
cat $foo | sed 's/eg/'$your_name'/g' > foo2;
echo Updating $foo;
mv foo2 $foo;
done

for foo in `find . -name '*sh'`;
do
cat $foo | sed 's/example/'$your_name'/g' > foo2;
echo Updating $foo;
mv foo2 $foo;
done

for foo in `find . -name '*sh'`;
do
cat $foo | sed 's/eg/'$your_name'/g' > foo2;
echo Updating $foo;
mv foo2 $foo;
done

# Part d: commit all the changes done so far:
git commit . -m "Changed the controller name from Example to "$your_name

# 4. Renames all the files in the directory from Example to <your_name>
# Part a: rename the cpp and hpp files:
for foo in `find . -name '*Example*pp'`;
do
echo $foo | sed 's/Example/'$cap_first_your_name'/g' > foo2;
echo git mv $foo `cat foo2`;
git mv $foo `cat foo2`;
done

# Part b: rename main file
your_cpp_file="scl_"$your_name"_main.cpp"
git mv scl_example_main.cpp `echo $your_cpp_file`

# Part c: commit the changes
git commit . -m "Changed the renamed files from Example to "$your_name

# 5. Compiles your new <your_name> application
your_executable_file="scl_"$your_name"_ctrl"
sh make_dbg.sh
sh make_rel.sh
chmod +x $your_executable_file
