#!/usr/bin/env python3
"""
Temporary shim to keep the original entrypoint working.
It now delegates to the modularized lawn_path_planner package.
"""

from lawn_path_planner.app import main


if __name__ == "__main__":
    main()
