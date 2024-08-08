# Copyright lowRISC Contributors.
# SPDX-License-Identifier: Apache-2.0

"""Sonata Test Runner

This script watches the UART output of the sonata system running a test. When
it encounters a recognised pass or fail message it will exit with the
appropriate error code.
"""

import argparse
import os
import shutil
import sys
import threading
import time
from enum import UNIQUE, IntEnum, verify
from glob import glob
from pathlib import Path
from queue import Queue
from subprocess import PIPE, Popen
from typing import Generator

import serial

PASSED_MESSAGE: str = "All tests finished"
FAILED_MESSAGE: str = "Test(s) Failed"
TICK_SECONDS: float = 0.01
SONATA_DRIVE_GLOBS: tuple[str, ...] = ("/run/media/**/SONATA",)
BAUD_RATE: int = 921_600


@verify(UNIQUE)
class ReturnCode(IntEnum):
    """The return codes of this command line tool."""

    TESTS_PASSED = 0
    TESTS_FAILED = 1
    BAD_INPUT = 2
    TIMEOUT = 3
    SIMULATOR_DIED = 4
    FPGA_FILESYSEM_NOT_FOUND = 5

    def __str__(self) -> str:
        match self:
            case self.TESTS_PASSED:
                return "tests passed"
            case self.TESTS_FAILED:
                return "tests failed"
            case self.BAD_INPUT:
                return "bad script user input"
            case self.TIMEOUT:
                return "tests timed out"
            case self.SIMULATOR_DIED:
                return "simulator died unexpectedly"
            case self.SIMULATOR_DIED:
                return "a mounted fpga filesystem could not be found"
            case _:
                raise NotImplementedError


return_code: Queue[ReturnCode] = Queue(maxsize=1)
"""This queue is used to return an error code from a thread to main."""
main_finished = threading.Event()
"""The main finished event is used to inform thread that main has finished."""


def find_sonata_drive() -> str:
    """Finds the location of sonata fpga's filesystem."""
    for pattern in SONATA_DRIVE_GLOBS:
        if directories := glob(pattern, recursive=True):  # noqa: PTH207
            return directories[0]
    raise FileNotFoundError


class Config(argparse.Namespace):
    """Configuration of the test runner.

    This class exists to provide a clear type associated with the set
    attributes derived from the command line interface.
    """

    def __init__(self) -> None:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "-t",
            "--timeout",
            type=float,
            help="Seconds before timing out. Defaults to no timeout.",
        )

        subparsers = parser.add_subparsers(required=True)

        fpga_parser = subparsers.add_parser("fpga", help="Run test on FPGA")
        fpga_parser.set_defaults(fpga=True)
        fpga_parser.add_argument("tty", type=str, help="tty device location")
        fpga_parser.add_argument(
            "-u",
            "--uf2-file",
            type=Path,
            default=Path(
                "./build/cheriot/cheriot/release/sonata_test_suite.uf2"
            ),
            help="Test suite uf2 file",
        )

        sim_parser = subparsers.add_parser("sim", help="Run test on simulator")
        sim_parser.set_defaults(fpga=False)
        sim_parser.add_argument(
            "-e",
            "--elf-file",
            type=Path,
            default=Path("./build/cheriot/cheriot/release/sonata_test_suite"),
            help="Test suite elf file",
        )
        sim_parser.add_argument(
            "--simulator-binary",
            type=Path,
            help="The location of the simulator binary",
        )
        sim_parser.add_argument(
            "--sim-boot-stub",
            type=Path,
            help="The simulator boot stub location",
        )
        sim_parser.add_argument(
            "--uart-log",
            type=Path,
            default=Path("uart0.log"),
            help="The uart log location",
        )

        # Set the attributs of this object from the program arguments
        parser.parse_args(namespace=self)

        if not self.fpga:
            if not self.simulator_binary:
                self.simulator_binary: Path = Path(
                    str(binary)
                    if (binary := shutil.which("sonata-simulator"))
                    else "./sonata-simulator"
                )
            if not self.sim_boot_stub:
                if stub := os.getenv("SONATA_SIM_BOOT_STUB"):
                    self.sim_boot_stub: Path = Path(stub)
                else:
                    print("No simulator boot stub found")
                    exit(ReturnCode.BAD_INPUT)

        for path in (
            (self.uf2_file, self.tty)
            if self.fpga
            else (self.simulator_binary, self.sim_boot_stub, self.elf_file)
        ):
            if not os.path.exists(path):  # noqa: PTH110
                print(f"'{path}' doesn't exist.")
                exit(ReturnCode.BAD_INPUT)


def run_simulator(config: Config) -> None:
    """Starts a simulator process.

    The process is killed if `main_finished` is set.
    """
    command = [
        config.simulator_binary,
        "-E",
        config.sim_boot_stub,
        "-E",
        config.elf_file,
    ]
    with Popen(command, stdout=PIPE, stderr=PIPE) as proc:
        while True:
            if (code := proc.poll()) is not None:
                print(f"Simulator Stopped with error code: {code}")
                for stdio in proc.communicate():
                    print(stdio.decode(sys.stdout.encoding))
                return_code.put(ReturnCode.SIMULATOR_DIED)
                break
            if main_finished.is_set():
                proc.terminate()
                break
            time.sleep(TICK_SECONDS)


def watch_output(config: Config) -> None:
    """Watches the output of either the simulator or the fpga.

    When a passed or failed message is observed it puts the appropriate return
    code into the return code queue. If no message is observed, this function
    will run forever, so should be run in a daemon thread.
    """

    def fpga_readlines() -> Generator[str, None, None]:
        """Reads each line from the fpga serial port."""
        with serial.Serial(config.tty, BAUD_RATE, timeout=2) as uart:
            while True:
                try:
                    line = uart.readline().decode(sys.stdout.encoding)
                except UnicodeDecodeError:
                    # Accept garbled data gracefully. Decode errors can often
                    # happen shortly after reloading the sonata's firmware.
                    line = "XXX Decode Error (This is expected at the start)\n"
                yield line

    def simulation_readlines() -> Generator[str, None, None]:
        """Reads each line from the simulator uart output log."""
        while True:
            try:
                with config.uart_log.open() as uart:
                    while True:
                        yield uart.readline()
            except FileNotFoundError:
                # Keep attempting to open the UART log file, until it exists.
                time.sleep(TICK_SECONDS)

    lines = simulation_readlines() if not config.fpga else fpga_readlines()
    for line in lines:
        sys.stdout.write(line)
        if PASSED_MESSAGE in line:
            return_code.put(ReturnCode.TESTS_PASSED)
            break
        if FAILED_MESSAGE in line:
            return_code.put(ReturnCode.TESTS_FAILED)
            break
        time.sleep(TICK_SECONDS)


def watchdog(config: Config) -> None:
    """Sleeps for the configured time before triggering a timeout."""
    time.sleep(config.timeout)
    return_code.put(ReturnCode.TIMEOUT)


def main(config: Config) -> None:
    """The main thread of the program."""
    # Remove any existing files that we don't want to exist
    # before the watch_output thread starts.
    if not config.fpga:
        config.uart_log.unlink(missing_ok=True)

    # Start watch_output threads
    threading.Thread(target=watch_output, args=(config,), daemon=True).start()

    if config.fpga:
        # Start the test running on the fpga by copying the firmware over to
        # it.
        try:
            fpga_drive = find_sonata_drive()
            shutil.copy(config.uf2_file, fpga_drive)
        except FileNotFoundError:
            return_code.put(ReturnCode.FPGA_FILESYSEM_NOT_FOUND)
    else:
        # Start the simulator in a seperate thread.
        simulator_thread = threading.Thread(
            target=run_simulator, args=(config,)
        )
        simulator_thread.start()

    if config.timeout:
        # Start watchdog threads
        threading.Thread(target=watchdog, args=(config,), daemon=True).start()

    # Wait for a return code to be put
    # in the return code queue
    code = return_code.get()
    # Tell the other threads main has finished
    main_finished.set()
    if code:
        print(code)
    exit(code)


if __name__ == "__main__":
    config = Config()
    main(config)
