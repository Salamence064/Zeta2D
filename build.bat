@echo off

@REM NOTE: you will need to change portions of this script
@REM       to match your directory names and project structure.
@REM       This is just a rough basis for how to build it.

@REM NOTE: If you wish to add this to your project's build system
@REM        you will need to add in an intermediary step of entering
@REM        the folder from the cloned repository before compiling
@REM        the object files.

@REM NOTE: You will also need to add any extra parameters to Zeta's
@REM        compilation 

echo "Build Starting"

@REM This should only be uncommented if you move the script out of ZETA's scope.
@REM if not exist "build" (
@REM     echo "Creating 'build' directory"
@REM     mkdir "build"
@REM )

@REM if not exist "build/vendor" (
@REM     echo "Creating 'build/vendor' directory"
@REM     mkdir "build/vendor"
@REM )

@REM if not exist "lib" (
@REM     echo "Creating 'lib' directory"
@REM     mkdir "lib"
@REM )


@REM check if the ZETA folder is empty
@REM if you want to add this part, uncomment and change 
@REM the zeta2d filepath to what your cloned directory is named
@REM dir /b /s /a "zeta2d/" | findstr . > nul || (
@REM     echo "Zeta2D folder is empty"
@REM     echo "Build Failed"
@REM     GOTO FAILED
@REM )

@REM check if Zeta2D has already been built
if exist "lib/zeta2d.dll" (
    GOTO SKIP_ZETA2D_BUILD
)


@REM ---------- Start of Zeta2D Build ----------

echo "Building Zeta2D"

if not exist "build/" (
    echo "Creating 'build' directory" 
    mkdir "build/"
)

if not exist "lib/" (
    echo "Creating 'lib' directory"
    mkdir "lib/"
)

@REM create the object and library files
pushd "build/"

    g++ -O3 -I../include -c ../src/*.cpp
    g++ -shared -Wl,-soname,../libzeta2d.dll -Wl,--out-implib,../lib/libzeta2d.a -o ../lib/zeta2d.dll *.o

popd @REM "build/"

@REM ----------- End of Zeta2D Build -----------


:SKIP_ZETA2D_BUILD

@REM This is where you would compile your own project were you to move the script.

echo "Build Complete"

:FAILED
