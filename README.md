# STM32F411E-IIR-Filter

This code was created for the STM32F411E DISCO board. The goal was to create an IIR digital low-pass filter that could filter out a contaminated signal. This contaminated signal would be downloaded directly onto the board at address 0x8020000.

There are two different ProcessSample() functions that can be commented in and out for filtering the signal: ProcessSample() and ProcessSampleMAC(). ProcessSample() executes the filter steps using regular C operations. ProcessSampleMAC() uses two SMLAD SIMD intrinsic and one SMLABB MAC command to calculate the filter steps.

There are two additional processing functions: CreateEcho() and CreateReverb(). These functions, which can be uncommented/commented to activate, add an echo or reverb effect to the processed signal. It should be noted that they cannot be simultaneously used. They both use circular buffers to optimize speed.
