# Meraki NANDloader

## Introduction

This project aims to provide a drop-in replacement for the boot-loaders
that are shipped on the Meraki MR18 and Meraki Z1.

## Requirements

This project needs Debian's MIPS-Crosscompiler suite for MIPS.
<https://wiki.debian.org/CrossCompiling>

furthermore cmake >= 3.12-ish is needed.

## Build

 cmake .

 make

 The "product" is in nandloader.fw

## Supported Hardware

Cisco Meraki MR18, Cisco Meraki Z1

## Options
By default, this will build the NANDloader for the MR18 with the
SGMII Calibration integrated. Use cmake-gui for configuration.
