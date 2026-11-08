# PSVR2 passthrough — OpenXR layer research

This repository tracks an **experimental PSVR2 passthrough patch** for the [xr_passthrough_layer project](https://github.com/yshui/xr_passthrough_layer). It is a working area for investigating camera passthrough integration with Linux/OpenXR tooling, not a packaged consumer driver.

## Repository contents

- `patches/psvr2-passthrough.patch.gz` — compressed patch used for the experiment.
- `.github/workflows/` — repository automation.

## Project scope

The work focuses on understanding a hardware/software integration problem: how PSVR2 camera imagery could be made available to an XR application through an OpenXR layer. Relevant areas include driver interoperability, device interfaces, debugging, and testing on Linux.

**Status:** experimental. This README does **not** claim that passthrough, headset tracking, or general PSVR2 compatibility is complete or validated. A working deployment requires separate testing against the matching upstream revision and hardware.

## Reviewing the patch

1. Inspect the patch before applying it:
   ```sh
   gzip -dc patches/psvr2-passthrough.patch.gz | less
   ```
2. Identify the intended upstream revision and review the changes before use. Do not assume this patch applies cleanly to the latest upstream version.
3. Test builds and functionality on a separate development setup.

## Attribution

This is downstream experimental work based on [yshui/xr_passthrough_layer](https://github.com/yshui/xr_passthrough_layer). Upstream code and third-party components remain subject to their respective licenses. This repository is not affiliated with Sony, Valve, or the upstream maintainers.
