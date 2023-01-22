
# Find clean script working directory
cd "$(dirname "$0")"
COMPILE_WD="$(pwd)/../build"
APPS_WD="$(pwd)/../build/apps"

# Change to the build directory
cd $COMPILE_WD

# Remove all files ignored by git (compile scripts, etc.)
git clean -dfX

# Go one layer deeper, into apps/
cd $APPS_WD

# Remove all files ignored by git (compile scripts, etc.)
git clean -dfX