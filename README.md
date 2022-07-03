# The Ray Tracer formerly known as CERES

## File Structure Description:
- `external/`: Collection of external third-party libraries used (but unedited) but this project
- `include/`: Public interface for the library (`.hpp`)
- `lib/`: Source for the ray tracer library.  Contains both implementation (`.cpp`) and private headers (`.hpp`)
- `src/`: Source for the application.  Contains both implementation (`.cpp`) and private headers (`.hpp`)

## Installation (Linux):
First clone the repository with all submodules:

```
git clone --recurse-submodules -j8 git@github.com:ceres-navigation/crt2.git
```

Run the setup script (to configure third party libraries):

```
./setup.sh
```
*NOTE: If `setup.sh` is not executable, simply run `chmod +x setup.sh` and try again.

To build the project simply run from the repository root directory:

```
mkdir build
cd build
cmake ..
make -j
```

This will produce the following executables:
- `convert_obj`: This will convert a .OBJ file from ASCII format to the ray tracer's compressed binary format for faster loading.  Use as: `./convert_obj <path to .obj> <output path>`
