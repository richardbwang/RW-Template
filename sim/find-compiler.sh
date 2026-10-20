# Finds a C++ compiler for this computer. Used by sim/run-sim.sh, sim/fit-profile.sh and
# tests/run-tests.sh after they move to the project folder. Sets CXX and CXXFLAGS.
CXX="${CXX:-c++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
  echo "No C++ compiler found (looked for \"$CXX\"). Install one, then run this again:"
  echo "  macOS:   xcode-select --install"
  echo "  Windows: type wsl in the terminal first (see the Simulator section of README.md)"
  echo "  Linux:   sudo apt install g++"
  exit 1
fi

# Some macOS Command Line Tools installs point the compiler at an SDK without the C++ headers.
# If <vector> can't be found, use the newest installed SDK that has them.
if ! printf '#include <vector>\nint main() { return 0; }\n' | "$CXX" -x c++ -fsyntax-only $CXXFLAGS - 2>/dev/null; then
  for sdk in $(ls -d /Library/Developer/CommandLineTools/SDKs/*.sdk 2>/dev/null | sort -r); do
    if [ -f "$sdk/usr/include/c++/v1/vector" ]; then
      CXXFLAGS="$CXXFLAGS -stdlib=libc++ -isysroot $sdk -isystem $sdk/usr/include/c++/v1"
      break
    fi
  done
fi
