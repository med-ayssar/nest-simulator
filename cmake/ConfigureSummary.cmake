# cmake/ConfigureSummary.cmake
#
# This file is part of NEST.
#
# Copyright (C) 2004 The NEST Initiative
#
# NEST is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# NEST is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with NEST.  If not, see <http://www.gnu.org/licenses/>.

function(NEST_PRINT_CONFIG_SUMMARY)
  # set summary color: here please choose appropriate color!
  message("${BoldCyan}")
  message("--------------------------------------------------------------------------------")
  message("NEST Configuration Summary")
  message("--------------------------------------------------------------------------------")
  message("")

  if(CMAKE_BUILD_TYPE)
    message("Build type          : ${CMAKE_BUILD_TYPE}")
  endif()

  message("Target System       : ${CMAKE_SYSTEM_NAME}")
  message("Cross Compiling     : ${CMAKE_CROSSCOMPILING}")
  message("C compiler          : ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION} (${CMAKE_C_COMPILER})")
  message("C compiler flags    : ${ALL_CFLAGS}")
  message("C++ compiler        : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} (${CMAKE_CXX_COMPILER})")
  message("C++ compiler flags  : ${ALL_CXXFLAGS}")
  message("Build dynamic       : ${BUILD_SHARED_LIBS}")

  message("")
  message("Built-in modules    : ${SLI_MODULES}")

  if(external-modules)
    message("User modules        : ${external-modules}")

    foreach(mod ${external-modules})
      message("  ${mod}:")
      message("    Header          : ${${mod}_EXT_MOD_INCLUDE}")
      message("    Library         : ${${mod}_EXT_MOD_LIBRARY}")
    endforeach()
  else()
    message("User modules        : None")
  endif()

  message("")

  if(HAVE_PYTHON)
    message("Python bindings     : Yes (Python ${Python_VERSION}: ${PYTHON})")
    message("    Includes        : ${Python_INCLUDE_DIRS}")
    message("    Libraries       : ${Python_LIBRARIES}")

    if(CYTHON_FOUND)
      message("    Cython          : Yes (Cython ${CYTHON_VERSION}: ${CYTHON_EXECUTABLE})")
    else()
      message("    Cython          : No. Make sure to cythonize `pynestkernel.pyx` manually.")
    endif()

    if(HAVE_MPI4PY)
      message("    MPI4Py          : Yes (${PY_MPI4PY}/include)")
    else()
      message("    MPI4Py          : No ")
    endif()
  else()
    message("Python bindings     : No")
  endif()

  message("")

  if(BUILD_DOCS)
    message("Documentation       : Yes")

    if(BUILD_SPHINX_DOCS)
      message("  Sphinx doc build  : Yes (${SPHINX_EXECUTABLE})")
    else()
      message("  Sphinx doc build  : No")
    endif()

    if(BUILD_DOXYGEN_DOCS)
      message("  Doxygen doc build : Yes (${DOXYGEN_EXECUTABLE})")
      message("    Graphviz        : Yes (${DOXYGEN_DOT_EXECUTABLE})")
    else()
      message("  Doxygen doc build : No")
    endif()
  else()
    message("Documentation       : No")
  endif()

  message("")

  if(OPENMP_FOUND)
    message("Use threading       : Yes (OpenMP: ${OpenMP_CXX_FLAGS})")
  else()
    message("Use threading       : No")
  endif()

  message("")

  if(HAVE_GSL)
    message("Use GSL             : Yes (GSL ${GSL_VERSION})")
    message("    Includes        : ${GSL_INCLUDE_DIRS}")
    message("    Libraries       : ${GSL_LIBRARIES}")
  else()
    message("Use GSL             : No")
  endif()

  message("")

  if(HAVE_READLINE)
    message("Use Readline        : Yes (GNU Readline ${READLINE_VERSION})")
    message("    Includes        : ${READLINE_INCLUDE_DIRS}")
    message("    Libraries       : ${READLINE_LIBRARIES}")
  else()
    message("Use Readline        : No")
  endif()

  message("")

  if(HAVE_LIBLTDL)
    message("Use libltdl         : Yes (LTDL ${LTDL_VERSION})")
    message("    Includes        : ${LTDL_INCLUDE_DIRS}")
    message("    Libraries       : ${LTDL_LIBRARIES}")
  else()
    message("Use libltdl         : No")

    if(APPLE AND with-ltdl AND NOT static-libraries)
      message("                    : (OS X does not provide libltdl with its libtool.")
      message("                    :  If you require dynamic loading of user modules, please")
      message("                    :  install GNU libtool, e.g. via `brew install libtool`.)")
    endif()
  endif()

  message("")

  if(HAVE_MPI)
    message("Use MPI             : Yes (MPI: ${MPI_CXX_COMPILER})")
    message("    Includes        : ${MPI_CXX_INCLUDE_PATH}")
    message("    Libraries       : ${MPI_CXX_LIBRARIES}")

    if(MPI_CXX_COMPILE_FLAGS)
      message("    Compile Flags   : ${MPI_CXX_COMPILE_FLAGS}")
    endif()

    if(MPI_CXX_LINK_FLAGS)
      message("    Link Flags      : ${MPI_CXX_LINK_FLAGS}")
    endif()

    set(MPI_LAUNCHER "${MPIEXEC} ${MPIEXEC_NUMPROC_FLAG} <np> ${MPIEXEC_PREFLAGS} <prog> ${MPIEXEC_POSTFLAGS} <args>")
    string(REPLACE "  " " " MPI_LAUNCHER ${MPI_LAUNCHER})
    message("    Launcher        : ${MPI_LAUNCHER}")
  else()
    message("Use MPI             : No")
  endif()

  message("")

  if(TIMER_DETAILED)
    message("Detailed timers     : Yes")
  else()
    message("Detailed timers     : No")
  endif()

  message("")

  if(HAVE_MUSIC)
    message("Use MUSIC           : Yes (MUSIC ${MUSIC_VERSION})")
    message("    Includes        : ${MUSIC_INCLUDE_DIRS}")
    message("    Libraries       : ${MUSIC_LIBRARIES}")
  else()
    message("Use MUSIC           : No")
  endif()

  message("")

  if(HAVE_LIBNEUROSIM)
    message("Use libneurosim     : Yes (LibNeurosim ${LIBNEUROSIM_VERSION})")
    message("    Includes        : ${LIBNEUROSIM_INCLUDE_DIRS}")
    message("    Libraries       : ${LIBNEUROSIM_LIBRARIES}")
  else()
    message("Use libneurosim     : No")
  endif()

  message("")

  if(HAVE_BOOST)
    message("Use Boost           : Yes (Boost ${BOOST_VERSION})")
    message("    Includes        : ${BOOST_INCLUDE_DIR}")
    message("    Libraries       : ${BOOST_LIBRARIES}")
  else()
    message("Use Boost           : No")
  endif()

  message("")

  if(HAVE_SIONLIB)
    message("Use SIONlib         : Yes (SIONlib ${SION_EXECUTABLE})")
    message("    Includes        : ${SIONLIB_INCLUDE}")
    message("    Libraries       : ${SIONLIB_LIBRARIES}")
  else()
    message("Use SIONlib         : No")
  endif()

  if(HAVE_HDF5)
    message("Use SONATA          : Yes (HDF5 ${HDF5_VERSION})")
    message("    Includes        : ${HDF5_INCLUDE_DIR}")
    message("    Libraries       : ${HDF5_LIBRARIES}")
    message("")
  else()
    message("Use SONATA          : No")
  endif()

  if(with-libraries)
    message("")
    message("Additional libraries:")

    foreach(lib ${with-libraries})
      message("                     ${lib}")
    endforeach()
  endif()

  if(with-includes)
    message("")
    message("Additional includes:")

    foreach(inc ${with-includes})
      message("                     -I${inc}")
    endforeach()
  endif()

  if(with-intel-compiler-flags AND NOT("${CMAKE_C_COMPILER_ID}" STREQUAL "Intel" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel"))
    message("")
    message("You have specified flags for the Intel compiler (-Dwith-intel-compiler-flags),")
    message("but the Intel compiler is not used. These flags are therefore ignored.")
  endif()

  if(HAVE_MPI)
    message("")
    message("For details on setting specific flags for your MPI launcher command, see the")
    message("CMake documentation at https://cmake.org/cmake/help/latest/module/FindMPI.html")
  endif()

  message("")
  message("--------------------------------------------------------------------------------")
  message("")
  message("The NEST executable will be installed to:")
  message("  ${CMAKE_INSTALL_FULL_BINDIR}/")
  message("")
  message("NEST dynamic libraries and user modules will be installed to:")
  message("  ${CMAKE_INSTALL_FULL_LIBDIR}/nest/")
  message("")

  if(BUILD_DOCS)
    message("Documentation and examples will be installed to:")
    message("  ${CMAKE_INSTALL_FULL_DOCDIR}/")
    message("")
  endif()

  if(HAVE_PYTHON)
    message("PyNEST will be installed to:")
    message("    ${CMAKE_INSTALL_PREFIX}/${PYEXECDIR}")
    message("")
  endif()

  message("To set necessary environment variables, add the following line")
  message("to your ~/.bashrc :")
  message("  source ${CMAKE_INSTALL_FULL_BINDIR}/nest_vars.sh")
  message("")
  message("--------------------------------------------------------------------------------")
  message("")

  if(NOT HAVE_GSL)
    message("")
    message("ATTENTION!")
    message("You are about to compile NEST without the GNU Scientific")
    message("Library or your GSL is too old (before v1.11). This means")
    message("that conductance-based neuron models and some random number")
    message("generators will not be available.")
    message("")
    message("--------------------------------------------------------------------------------")
    message("")
  endif()

  message("You can now build and install NEST with")
  message("  make")
  message("  make install")
  message("  make installcheck")
  message("")

  message("If you experience problems with the installation or the use of NEST,")
  message("please see https://www.nest-simulator.org/frequently_asked_questions")
  message("or go to https://www.nest-simulator.org/community to find out how to")
  message("join the user mailing list.")

  # reset output color
  message("${Reset}")
endfunction()
