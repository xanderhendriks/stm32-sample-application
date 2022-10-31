STM32 Sample Project
====================

Introduction
------------

A simple STM32 sample project how to use COntinuous Integration (CI) for building binaries and documentation created from source code.
The project is created for the `B-L4S5I-IOT01A STM32L4+ Discovery kit IoT node <https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html>`_.
It currently only contains the automatically generated code and an example crc.c for the doxygen documentation extraction.

Depending upon the amount of interest I might add `CppUTest <http://cpputest.github.io/>`_ unit testing, both on amd64 and on target. The on target test will use
a `Raspberry Pi <https://www.raspberrypi.org/>`_ for programming and monitoring test execution.

Next to this most of my projects use a system test based on `PyTest <https://docs.pytest.org/>`_ to test the complete binary. I could add an example of that as well.

Please let me know in the `issues <https://github.com/xanderhendriks/stm32-sample-project/issues>`_ what you think. I'm also happy for improvements ideas to the current setup as well.
Next to Continuous Integration we also have to do a lot of Continuous Learning.