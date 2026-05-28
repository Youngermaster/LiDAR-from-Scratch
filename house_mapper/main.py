"""
Entry point for the house_mapper subproject.

Usage:
    python main.py
    python main.py --port /dev/cu.usbserial-1130
    python main.py --replay scan_20260522_180342.csv

This file lives at the project root so that running it from inside
this directory adds the directory to sys.path, which makes
`import house_mapper.app` resolve against the local package.

The env-var dance below must happen BEFORE pygame is imported, which
is why it cannot live inside the package's `app` module: importing
`house_mapper.app` triggers `import pygame`, and pygame prints its
welcome banner during import unless this flag is already set.
"""

from __future__ import annotations

import os
import sys

# Silence pygame's "Hello from the pygame community" banner.
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

from house_mapper.app import main  # noqa: E402


if __name__ == "__main__":
    sys.exit(main())
