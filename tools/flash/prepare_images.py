#!/usr/bin/env python3
"""Merge the SDK-selected sysbuild images once per nRF5340 core."""

import argparse
from pathlib import Path
import sys

try:
    from intelhex import IntelHex, IntelHexError
    import yaml
except ImportError:
    sys.exit("Use the SDK Python environment, or install: python -m pip install intelhex PyYAML")


def load_yaml(path):
    with path.open(encoding="utf-8") as source:
        return yaml.safe_load(source)


def prepare_images(build_dir, output_dir, fota=False):
    build_dir = Path(build_dir).resolve()
    domains = load_yaml(build_dir / "domains.yaml")
    app_name = domains["default"]
    names = {entry["name"] for entry in domains["domains"]}
    required = {app_name, "ipc_radio"}
    if fota:
        required.update(("mcuboot", "b0n"))
    if names != required:
        raise ValueError(f"Expected {'FOTA' if fota else 'standard'} domains {sorted(required)}, "
                         f"found {sorted(names)}. Rebuild with the matching configuration.")

    app_config = (build_dir / app_name / "zephyr/.config").read_text().splitlines()
    for option in ("CONFIG_BOOTLOADER_MCUBOOT", "CONFIG_AUDIO_BT_MGMT_DFU"):
        if (f"{option}=y" in app_config) != fota:
            raise ValueError(f"{option} does not match the requested build type")

    merged = {"CP_APPLICATION": IntelHex(), "CP_NETWORK": IntelHex()}
    for name in sorted(required):
        zephyr_dir = build_dir / name / "zephyr"
        # runners.yaml selects the signed radio image and provisioned b0n image,
        # not the unsigned zephyr.hex files alongside their ELF files.
        runner = load_yaml(zephyr_dir / "runners.yaml")
        image = Path(runner["config"]["hex_file"])
        if not image.is_absolute():
            image = zephyr_dir / image
        image = image.resolve()
        config = (zephyr_dir / ".config").read_text().splitlines()
        app = "CONFIG_SOC_NRF5340_CPUAPP=y" in config
        net = "CONFIG_SOC_NRF5340_CPUNET=y" in config
        if app == net:
            raise ValueError(f"Cannot determine nRF5340 core for {name}")
        core = "CP_APPLICATION" if app else "CP_NETWORK"
        if (name in (app_name, "mcuboot")) != app:
            raise ValueError(f"Unexpected core for {name}: {core}")
        data = IntelHex(str(image))
        low, high = (0, 0x100000) if app else (0x1000000, 0x1040000)
        if not data.segments() or any(a < low or b > high for a, b in data.segments()):
            raise ValueError(f"{image} contains no flash data or data outside {core} flash")
        merged[core].merge(data, overlap="error")
        print(f"{core}: {image}")

    # All inputs are validated before producing anything used by the flasher.
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    merged["CP_APPLICATION"].write_hex_file(str(output_dir / "merged.hex"))
    merged["CP_NETWORK"].write_hex_file(str(output_dir / "merged_CPUNET.hex"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--fota", action="store_true")
    args = parser.parse_args()
    try:
        prepare_images(args.build_dir, args.output_dir, args.fota)
    except (OSError, ValueError, KeyError, TypeError, IntelHexError, yaml.YAMLError) as error:
        parser.exit(1, f"Cannot prepare flash images: {error}\n")


if __name__ == "__main__":
    main()
