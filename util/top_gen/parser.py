# Copyright lowRISC contributors
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Top configuration structures and deserialiser from TOML files."""

import json
from enum import Enum
from pathlib import Path
from typing import Any

import toml
from pydantic import BaseModel, field_validator, model_validator
from pydantic.dataclasses import dataclass
from typing_extensions import Self


class Direction(str, Enum):
    INOUT = "inout"
    INPUT = "input"
    OUTPUT = "output"

    def __and__(self, other: "Direction") -> "Direction":
        match (self, other):
            case (Direction.INPUT, Direction.INPUT):
                return self
            case (Direction.OUTPUT, Direction.OUTPUT):
                return self
            case _:
                return Direction.INOUT

    @staticmethod
    def from_rdl_sigtype(sigtype: str) -> "Direction":
        match sigtype:
            case "PadInOut":
                return Direction("inout")
            case "PadInput":
                return Direction("input")
            case "PadOutput":
                return Direction("output")
        raise Exception("No type")


class BlockIoCombine(str, Enum):
    AND = "and"
    OR = "or"
    MUX = "mux"


class BlockIo(BaseModel, frozen=True):
    name: str
    type: Direction
    combine: BlockIoCombine | None = None
    default: int = 0
    length: int | None = None

    @model_validator(mode="after")
    def verify_block(self) -> Self:
        assert (
            self.type != Direction.INOUT or self.default == 0
        ), "An inout block IO cannot have a default value other than 0."
        return self


class Block(BaseModel, frozen=False):
    name: str
    instances: int
    ios: list[BlockIo] = []
    memory_start: int = 0
    memory_size: int = 0
    xbar: dict[str, str] = {}
    muxed_instances: int = 0

    @field_validator("instances")
    @staticmethod
    def check_instances(instances: int) -> int:
        if instances < 1:
            raise ValueError("Must have one or more instances of a block.")
        return instances


@dataclass(frozen=True)
class BlockIoUid:
    block: str
    instance: int
    io: str
    io_index: int | None = None
    """Used to index into an arrayed block IO."""


class Pin(BaseModel, frozen=True):
    name: str
    length: int | None = None
    """If not None, this is grouping of multiple pins with the given length."""
    block_ios: list[BlockIoUid]
    no_default_out: bool = False

    @model_validator(mode="after")
    def verify_pin(self) -> Self:
        if self.length is not None and not all(
            isinstance(block.io_index, int) for block in self.block_ios
        ):
            raise ValueError(
                "A pin grouping can only link to a block io array, i.e. "
                "when a length is specified all block_ios.io must be integers."
            )
        return self


class TopConfig(BaseModel, frozen=False):
    blocks: list[Block]
    pins: list[Pin]

    @field_validator("pins")
    @staticmethod
    def check_pins(pins: list[Pin]) -> list[Pin]:
        all_names = {pin.name for pin in pins}
        if len(all_names) != len(pins):
            raise ValueError("All pins must have unique names.")
        return pins

    def get_pin(self, name: str) -> Pin:
        try:
            return next(pin for pin in self.pins if pin.name == name)
        except StopIteration:
            print(f"A pin named '{name}' could not be found.")
            exit(3)


def next_power_of_two(x: int) -> int:
    return 1 << (x - 1).bit_length()


def _make_ios(device: dict[str, Any]) -> list[dict[str, str]]:
    ios = []
    for pad in device.get("pads", []):
        # Todo: Try to remove the thanslation.
        io = {
            "name": pad["name"],
            "length": pad.get("width", 1),
        }
        if type_ := pad["type"]:
            io["type"] = Direction.from_rdl_sigtype(type_).value
        if val := pad.get("combine"):
            io["combine"] = val.lower()
        ios.append(io)
    return ios


def parse_top_config(cfg_file: Path, rdl_file: Path) -> TopConfig:
    """Load the top configuration from the given TOML file."""
    with cfg_file.open() as file:
        config_dict = toml.load(file)

    with rdl_file.open() as file:
        rdl_dict = json.load(file)

    for device in rdl_dict["devices"]:
        if device["type"] == "mem":
            instances = 1
            mem_start = device["offset"]
        else:
            instances = len(device["offsets"])
            mem_start = device["offsets"][0]
        size = next_power_of_two(device.get("size", 0x1000))

        obj = {
            "name": device["name"].lower(),
            "instances": instances,
            "memory_start": mem_start,
            "memory_size": size,
            "ios": _make_ios(device),
        }
        if udps := device.get("udps"):
            if "xbar" in udps:
                obj["xbar"] = {
                    item["name"]: item["value"] for item in udps["xbar"]
                }

            if "clk_input" in udps:
                obj.setdefault("xbar", {})["clock"] = '"{}"'.format(
                    udps["clk_input"][0]
                )

            if "rst_input" in udps:
                obj.setdefault("xbar", {})["reset"] = '"{}"'.format(
                    udps["rst_input"][0]
                )

        config_dict.setdefault("blocks", []).append(obj)

    # Sort the blocks by the start address.
    Path("/tmp/log_top_config.txt").write_text(str(TopConfig(**config_dict)))
    config_dict["blocks"].sort(key=lambda b: b["memory_start"])
    return TopConfig(**config_dict)
