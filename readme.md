***********************************************************

This repository contains code for encrypted PID controllers
implemented with labeled homomorphic encryption, the Joye-Libert
Cryptosystem, and the HC-128 stream cipher.

The implementations of HC-128, Joye-Libert, and labeled
homomorphic encryption have been taken from the
CryptoToolbox repository: https://github.com/pettsol/CryptoToolbox

The implementation requires the GNU Multiple Integer Precision
Library (GMP library) to be installed.

The sample programs have been compiled and tested using g++
9.3.0 on an Ubuntu 20.04 system.

Proportional control sample program:

g++ p_test.cpp labeled_he_control.cpp LabeledHE/labeled_he.cpp HC-128/hc128.cpp joye_libert_journal/joye_libert.cpp -o p_test -lgmp

Proportional-integral control sample program:

g++ pi_test.cpp labeled_he_control.cpp LabeledHE/labeled_he.cpp joye_libert_journal/joye_libert.cpp HC-128/hc128.cpp -o pi_test -lgmp

Proportional-integral-derivative control sample program:

g++ pid_test.cpp labeled_he_control.cpp LabeledHE/labeled_he.cpp joye_libert_journal/joye_libert.cpp HC-128/hc128.cpp -o pid_test -lgmp

Petter Solnoer - 28/03/2022

*************************************************************
