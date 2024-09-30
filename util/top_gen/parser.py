# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Top configuration structures and deserialiser from TOML files."""

from enum import Enum
from pathlib import Path

import toml
from pydantic import BaseModel, field_validator, model_validator
from typing_extensions import Self


class BlockIoType(str, Enum):
    INOUT = "inout"
    INPUT = "input"
    OUTPUT = "output"


class BlockIoCombine(str, Enum):
    AND = "and"
    OR = "or"
    MUX = "mux"


class BlockIo(BaseModel, frozen=True):
    name: str
    type: BlockIoType
    combine: BlockIoCombine | None = None
    default: int = 0
    length: int | None = None

    @model_validator(mode="after")
    def verify_square(self) -> Self:
        assert (
            self.type != BlockIoType.INOUT or self.default == 0
        ), "An inout block IO cannot have a default value other than 0."
        return self


class Block(BaseModel, frozen=True):
    name: str
    instances: int
    ios: list[BlockIo]

    @field_validator("instances")
    @staticmethod
    def check_pins(instances: int) -> int:
        if instances < 1:
            raise ValueError("Must have one or more instances of a block.")
        return instances


class PinBlockIoPointer(BaseModel, frozen=True):
    block: str
    instance: int
    io: str | int


class Pin(BaseModel, frozen=True):
    name: str
    length: int | None = None
    block_ios: list[PinBlockIoPointer]


class TopConfig(BaseModel, frozen=True):
    blocks: list[Block]
    pins: list[Pin]

    @field_validator("pins")
    @staticmethod
    def check_pins(pins: list[Pin]) -> list[Pin]:
        all_names = {pin.name for pin in pins}
        if len(all_names) != len(pins):
            raise ValueError("All pins must have unique names.")
        return pins

    def get_block(self, name: str) -> Block:
        try:
            return next(block for block in self.blocks if block.name == name)
        except StopIteration:
            print(f"A '{name}' block could not be found.")
            exit(3)

    def get_pin(self, name: str) -> Pin:
        try:
            return next(pin for pin in self.pins if pin.name == name)
        except StopIteration:
            print(f"A pin named '{name}' could not be found.")
            exit(3)


def parse_top_config(filename: Path) -> TopConfig:
    """Load the top configuration from the given TOML file."""
    with filename.open() as file:
        config_dict = toml.load(file)
    return TopConfig(**config_dict)
