STM32 Sample Project
====================

Introduction
------------

A simple STM32 sample project how to use Continuous Integration (CI) for building binaries and documentation created from source code.
The project is created for the `NUCLEO-F303K8 <https://www.st.com/en/evaluation-tools/nucleo-f303k8.html>`_.
It currently only contains the automatically generated code and an example crc.c for the doxygen documentation extraction.

Depending upon the amount of interest I might add `CppUTest <http://cpputest.github.io/>`_ unit testing, both on amd64 and on target. The on target test will use
a `Raspberry Pi <https://www.raspberrypi.org/>`_ for programming and monitoring test execution.

Next to this most of my projects use a system test based on `PyTest <https://docs.pytest.org/>`_ to test the complete binary. I could add an example of that as well.

Please let me know in the `issues <https://github.com/xanderhendriks/stm32-sample-application/issues>`_ what you think. I'm also happy for improvements ideas to the current setup as well.
Next to Continuous Integration we also have to do a lot of Continuous Learning.

Build pipeline
--------------

The build pipeline is automaticaly started when performing a build on master. In this case it will also tag the build with the next version number and create a relase.

After creating a pull request for every commit a build is executed with the version number 0.0.0 and a 10 character githash. If a test relase is required than the build pipeline can 
be run manually. If release is set to 'y' then it will create a release with the name of the 10 character hash postfixed with -dev.  

Tool suite
----------

The project was started with the STM32CubeIde, but this has since been migrated to a CMake based build system. This allows for use of VSCode with a devcontainer 
based on a docker image which can be used both for local development as well as in the CI pipeline.

The github actions shows how to build both types of projects, so people can choose what they prefer. I would recommend using STM32Cube extension with cmake files 
as it is more scalable and easier to maintain. Using the devcontainer also makes sure that everyone uses the same tool versions and new developers can have their
first build running in less than 10 minutes.

STM32CubeIde
^^^^^^^^^^^^

The project can be opened in STM32CubeIde by opening the .project file in the root of the repository. When using the IDE the following files can be deleted:
- cmake directory
- stm32/CMakeLists.txt
- stm32/CMakePresets.json
- stm32/.settings directory
- stm32/.vscode directory

STM32CubeIde can also be executed from a docker container. See the `stm32cubeide docker <https://hub.docker.com/repository/docker/xanderhendriks/stm32cubeide/general>`_.


STM32Cube - Extension
^^^^^^^^^^^^^^^^^^^^^

The project can also be opened in VSCode using a devcontainer with the `STM32Cube Extension <https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension>`_
using the `stm32cube-extension docker <https://hub.docker.com/repository/docker/xanderhendriks/stm32cube-extension/general>`_

To open the project in VSCode with the STM32Cube extension first open the repository's local top level directory. Then open the command palette (F1) and select 
'Remote-Containers: Open Folder in Container...'. Finally open the 'stm32' directory. Now the STM32Cube extension can be used to build and program the project.

When using the extension the following files can be deleted:
- .project
- .cproject

Troubleshooting
---------------

j-link udev configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

If the ST-Link can't be connected to from the Docker container, you may need to set up udev rules on the host system to allow access to the device.
Download the `st-link-udev-rules <https://github.com/AideTechBot/st-link-udev-rules>`_ files 49-stlinkv2.rules and 49-stlink*.rules and place them 
in the /etc/udev/rules.d/ directory on the host. After that execute the commands::

    sudo udevadm control --reload-rules
    sudo udevadm trigger

